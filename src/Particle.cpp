/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#include "emcl/Particle.h"
#include <cmath>

namespace emcl {



Particle::Particle(double x, double y, double t, double w) : p_(x, y, t)
{
	w_ = w;
}

double Particle::likelihood(LikelihoodFieldMap *map, Scan &scan)
{
	double ans = 0.0;
	for(int i=0;i<scan.ranges_.size();i+=scan.scan_increment_){
		if(not scan.valid(scan.ranges_[i]))
			continue;

		double lidar_x = p_.x_ + scan.lidar_pose_x_*cos(p_.t_) - scan.lidar_pose_y_*sin(p_.t_);
		double lidar_y = p_.y_ + scan.lidar_pose_x_*sin(p_.t_) + scan.lidar_pose_y_*cos(p_.t_);
		double lidar_t = p_.t_ + scan.lidar_pose_yaw_;

		double ang = scan.angle_min_ + i*scan.angle_increment_;
		double lx = lidar_x + scan.ranges_[i] * cos(ang + lidar_t);
		double ly = lidar_y + scan.ranges_[i] * sin(ang + lidar_t);

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
