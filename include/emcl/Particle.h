//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later

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

	bool checkWallConflict(LikelihoodFieldMap *map, double ox, double oy, 
			double range, uint16_t direction, double threshold, bool replace);

	void sensorReset(double ox, double oy,
		double range1, uint16_t direction1, double hit_lx1, double hit_ly1,
		double range2, uint16_t direction2, double hit_lx2, double hit_ly2);
};

}

#endif
