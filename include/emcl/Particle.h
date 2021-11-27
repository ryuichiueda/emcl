/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#ifndef PARTICLE_H__
#define PARTICLE_H__

#include "emcl/Pose.h"
#include "emcl/LikelihoodFieldMap.h"

namespace emcl {



class Particle
{
public:
	Particle(double x, double y, double t, double w);

	double likelihood(LikelihoodFieldMap *map, Scan &scan);
	bool wallConflict(LikelihoodFieldMap *map, Scan &scan, double threshold, bool replace);
	Pose p_;
	double w_;

	Particle operator =(const Particle &p);
private:
	bool isPenetrating(double ox, double oy, double range, uint16_t direction, 
			LikelihoodFieldMap *map, double &hit_lx, double &hit_ly);
};

}

#endif
