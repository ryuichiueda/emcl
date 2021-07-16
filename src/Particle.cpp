/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#include "emcl/Particle.h"
#include "emcl/ParticleFilter.h"
#include <cmath>

namespace emcl {



Particle::Particle(double x, double y, double t, double w) : p_(x, y, t)
{
	w_ = w;
}

double Particle::likelihood(LikelihoodFieldMap *map, Scan &scan)
{
	uint16_t t = p_.get16bitRepresentation();
	double lidar_x = p_.x_ + scan.lidar_pose_x_*ParticleFilter::cos_[t] 
				- scan.lidar_pose_y_*ParticleFilter::sin_[t];
	double lidar_y = p_.y_ + scan.lidar_pose_x_*ParticleFilter::sin_[t] 
				+ scan.lidar_pose_y_*ParticleFilter::cos_[t];
	uint16_t lidar_yaw = Pose::get16bitRepresentation(scan.lidar_pose_yaw_);

	uint16_t directions[scan.ranges_.size()];
	for(int i=0;i<scan.ranges_.size();i+=scan.scan_increment_){
		directions[i] = Pose::get16bitRepresentation(scan.angle_min_ + i*scan.angle_increment_);
	}

	double ans = 0.0;
	for(int i=0;i<scan.ranges_.size();i+=scan.scan_increment_){
		if(not scan.valid(scan.ranges_[i]))
			continue;
		uint16_t a = directions[i] + t + lidar_yaw;
		double lx = lidar_x + scan.ranges_[i] * ParticleFilter::cos_[a];
		double ly = lidar_y + scan.ranges_[i] * ParticleFilter::sin_[a];

		ans += map->likelihood(lx, ly);
	}
	return ans;
}

Particle Particle::operator =(const Particle &p)
{
	p_ = p.p_;
	w_ = p.w_;
	return *this;
}

}
