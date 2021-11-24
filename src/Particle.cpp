/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#include "emcl/Particle.h"
#include "emcl/Mcl.h"
#include <cmath>

namespace emcl {



Particle::Particle(double x, double y, double t, double w) : p_(x, y, t)
{
	w_ = w;
}

double Particle::likelihood(LikelihoodFieldMap *map, Scan &scan)
{
	uint16_t t = p_.get16bitRepresentation();
	double lidar_x = p_.x_ + scan.lidar_pose_x_*Mcl::cos_[t] 
				- scan.lidar_pose_y_*Mcl::sin_[t];
	double lidar_y = p_.y_ + scan.lidar_pose_x_*Mcl::sin_[t] 
				+ scan.lidar_pose_y_*Mcl::cos_[t];
	uint16_t lidar_yaw = Pose::get16bitRepresentation(scan.lidar_pose_yaw_);

	double ans = 0.0;
	for(int i=0;i<scan.ranges_.size();i+=scan.scan_increment_){
		if(not scan.valid(scan.ranges_[i]))
			continue;
		uint16_t a = scan.directions_16bit_[i] + t + lidar_yaw;
		double lx = lidar_x + scan.ranges_[i] * Mcl::cos_[a];
		double ly = lidar_y + scan.ranges_[i] * Mcl::sin_[a];

		ans += map->likelihood(lx, ly);
	}
	return ans;
}

bool Particle::isPenetrating(LikelihoodFieldMap *map, Scan &scan, double threshold, bool replace)
{
	uint16_t t = p_.get16bitRepresentation();
	double lidar_x = p_.x_ + scan.lidar_pose_x_*Mcl::cos_[t]
				- scan.lidar_pose_y_*Mcl::sin_[t];
	double lidar_y = p_.y_ + scan.lidar_pose_x_*Mcl::sin_[t]
				+ scan.lidar_pose_y_*Mcl::cos_[t];
	uint16_t lidar_yaw = Pose::get16bitRepresentation(scan.lidar_pose_yaw_);

	uint16_t t_delta = Pose::get16bitRepresentation(10.0/180 * M_PI);

	double ans = 0.0;
	int hit_counter = 0;
	for(int i=0;i<scan.ranges_.size();i+=scan.scan_increment_){
		if(not scan.valid(scan.ranges_[i]))
			continue;

		double range = scan.ranges_[i];
		uint16_t a = scan.directions_16bit_[i] + t + lidar_yaw;

		double hit_lx, hit_ly;
		if(isPenetrating(lidar_x, lidar_y, range, a, map, hit_lx, hit_ly))
			hit_counter++;
		else
			hit_counter = 0;

		if(hit_counter*scan.angle_increment_ >= threshold){
			if(replace){
				p_.x_ -= lidar_x + range * Mcl::cos_[a] - hit_lx;
				p_.y_ -= lidar_y + range * Mcl::sin_[a]- hit_ly;
			}
			return true;
		}
	}
	return false;
}

bool Particle::isPenetrating(double ox, double oy, double range, uint16_t direction,
		LikelihoodFieldMap *map, double &hit_lx, double &hit_ly)
{
	bool hit = false;
	for(double d=map->resolution_;d<range;d+=map->resolution_){
		double lx = ox + d * Mcl::cos_[direction];
		double ly = oy + d * Mcl::sin_[direction];

		if((not hit) and map->likelihood(lx, ly) > 0.99){
			hit = true;
			hit_lx = lx;
			hit_ly = ly;
		}
		else if(hit and map->likelihood(lx, ly) == 0.0){ // openspace after hit
			return true; // penetration
		}
	}
	return false;
}

Particle Particle::operator =(const Particle &p)
{
	p_ = p.p_;
	w_ = p.w_;
	return *this;
}

}
