/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#include "emcl/ExpResetMcl2.h"
#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <cmath>

namespace emcl {

ExpResetMcl2::ExpResetMcl2(const Pose &p, int num, const Scan &scan,
				const std::shared_ptr<OdomModel> &odom_model,
				const std::shared_ptr<LikelihoodFieldMap> &map,
				double alpha_th, 
				double expansion_radius_position, double expansion_radius_orientation,
				double extraction_rate, double range_threshold, bool sensor_reset)
	: alpha_threshold_(alpha_th), 
	  expansion_radius_position_(expansion_radius_position),
	  expansion_radius_orientation_(expansion_radius_orientation),
	  extraction_rate_(extraction_rate),
	  range_threshold_(range_threshold),
	  sensor_reset_(sensor_reset),
	  Mcl::Mcl(p, num, scan, odom_model, map)
{
}

ExpResetMcl2::~ExpResetMcl2()
{
}

void ExpResetMcl2::sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv)
{
	if(processed_seq_ == scan_.seq_)
		return;

	Scan scan;
	int seq = -1;
	while(seq != scan_.seq_){//trying to copy the latest scan before next 
		seq = scan_.seq_;
		scan = scan_;
	}

	scan.lidar_pose_x_ = lidar_x;
	scan.lidar_pose_y_ = lidar_y;
	scan.lidar_pose_yaw_ = lidar_t;

	int i = 0;
	if(!inv){
		for(auto e : scan.ranges_)
			scan.directions_16bit_.push_back(
				Pose::get16bitRepresentation(scan.angle_min_ + (i++)*scan.angle_increment_)
			);
	}else{
		for(auto e : scan.ranges_)
			scan.directions_16bit_.push_back(
				Pose::get16bitRepresentation(scan.angle_max_ - (i++)*scan.angle_increment_)
			);
	}

	double valid_pct = 0.0;
	int valid_beams = scan.countValidBeams(&valid_pct);
	if(valid_beams == 0)
		return;

	for(auto &p : particles_)
		p.w_ *= p.likelihood(map_.get(), scan);

	alpha_ = nonPenetrationRate( (int)(particles_.size()*extraction_rate_), map_.get(), scan);
	ROS_INFO("ALPHA: %f / %f", alpha_, alpha_threshold_);
	if(alpha_ < alpha_threshold_){
		ROS_INFO("RESET");
		expansionReset();
		for(auto &p : particles_)
			p.w_ *= p.likelihood(map_.get(), scan);
	}

	if(normalizeBelief() > 0.000001)
		resampling();
	else
		resetWeight();

	processed_seq_ = scan_.seq_;
}

double ExpResetMcl2::nonPenetrationRate(int skip, LikelihoodFieldMap *map, Scan &scan)
{
	static uint16_t shift = 0;
	int counter = 0;
	int penetrating = 0;
	for(int i=shift%skip; i<particles_.size(); i+=skip){
		counter++;
		if(particles_[i].isPenetrating(map, scan, range_threshold_, sensor_reset_))
			penetrating++;
	}
	shift++;

	std::cout << penetrating << " " << counter << std::endl;
	return (double)(counter - penetrating) / counter;
}

void ExpResetMcl2::expansionReset(void)
{
	for(auto &p : particles_){
		double length = 2*((double)rand()/RAND_MAX - 0.5)*expansion_radius_position_;
		double direction = 2*((double)rand()/RAND_MAX - 0.5)*M_PI;

		p.p_.x_ += length*cos(direction);
		p.p_.y_ += length*sin(direction);
		p.p_.t_ += 2*((double)rand()/RAND_MAX - 0.5)*expansion_radius_orientation_;
		p.w_ = 1.0/particles_.size();
	}
}

}
