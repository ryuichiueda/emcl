/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#include "mcl/Particle.h"
#include <cmath>
using namespace std;

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

		double ang = scan.angle_min_ + i*scan.angle_increment_;
		double lx = p_.x_ + scan.ranges_[i] * cos(ang + p_.t_);
		double ly = p_.y_ + scan.ranges_[i] * sin(ang + p_.t_);

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
