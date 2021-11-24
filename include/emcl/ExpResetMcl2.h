/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#ifndef EXP_PF2_H__
#define EXP_PF2_H__

#include "emcl/Mcl.h"
/*
#include <vector>
#include <sstream>
#include <random>

#include "emcl/Particle.h"
#include "emcl/OdomModel.h"
#include "emcl/LikelihoodFieldMap.h"

#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
*/

namespace emcl {

class ExpResetMcl2 : public Mcl
{
public: 
	ExpResetMcl2(const Pose &p, int num, const Scan &scan,
			const std::shared_ptr<OdomModel> &odom_model,
			const std::shared_ptr<LikelihoodFieldMap> &map,
			double alpha_th,
			double expansion_radius_position, double expansion_radius_orientation,
			double extraction_rate, double successive_penetration_threshold,
			bool sensor_reset);
	~ExpResetMcl2();

	void sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv);
private:
	double alpha_threshold_;
	double expansion_radius_position_;
	double expansion_radius_orientation_;

	double extraction_rate_;
	double range_threshold_;
	bool sensor_reset_;

	void expansionReset(void);

//bool Particle::isPenetrating(
	double nonPenetrationRate(int skip, LikelihoodFieldMap *map, Scan &scan);
};

}

#endif
