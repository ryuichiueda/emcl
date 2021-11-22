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

	double ans = 0.0;
	for(int i=0;i<scan.ranges_.size();i+=scan.scan_increment_){
		if(not scan.valid(scan.ranges_[i]))
			continue;

		uint16_t a = scan.directions_16bit_[i] + t + lidar_yaw;
		double lx = lidar_x + scan.ranges_[i] * ParticleFilter::cos_[a];
		double ly = lidar_y + scan.ranges_[i] * ParticleFilter::sin_[a];

		ans += map->likelihood(lx, ly);
	}
	return ans;
}

bool Particle::penetrationCheck(LikelihoodFieldMap *map, Scan &scan)
{
	uint16_t t = p_.get16bitRepresentation();
	double lidar_x = p_.x_ + scan.lidar_pose_x_*ParticleFilter::cos_[t] 
				- scan.lidar_pose_y_*ParticleFilter::sin_[t];
	double lidar_y = p_.y_ + scan.lidar_pose_x_*ParticleFilter::sin_[t] 
				+ scan.lidar_pose_y_*ParticleFilter::cos_[t];
	uint16_t lidar_yaw = Pose::get16bitRepresentation(scan.lidar_pose_yaw_);

	double ans = 0.0;
	for(int i=0;i<scan.ranges_.size();i+=scan.scan_increment_){
		if(not scan.valid(scan.ranges_[i]))
			continue;

		double range = scan.ranges_[i];
		uint16_t a = scan.directions_16bit_[i] + t + lidar_yaw;

		bool hit = false;
		double hit_lx = 0.0;
		double hit_ly = 0.0;
		for(double d=0.0;d<range;d+=map->resolution_){
			double lx = lidar_x + d * ParticleFilter::cos_[a];
			double ly = lidar_y + d * ParticleFilter::sin_[a];

			if(map->likelihood(lx, ly) > 0.99){
				hit = true;
				hit_lx = lx;
				hit_ly = ly;
			}
			else if(hit and map->likelihood(lx, ly) == 0.0){ // openspace after hit
				/* a kind of sensor reset */
				p_.x_ -= lx - hit_lx;
				p_.y_ -= ly - hit_ly;

				return true; // penetration
			}
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
