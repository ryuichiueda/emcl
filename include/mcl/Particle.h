/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#ifndef PARTICLE_H__
#define PARTICLE_H__

#include "mcl/Pose.h"
#include "mcl/LikelihoodFieldMap.h"
using namespace std;

class Particle
{
public:
	Particle(double x, double y, double t, double w);

	double likelihood(LikelihoodFieldMap *map, const vector<double> &ranges,
		double angle_min, double angle_increment);
	Pose p_;
	double w_;

	Particle operator =(const Particle &p);
};

#endif
