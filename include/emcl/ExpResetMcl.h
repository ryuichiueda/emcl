/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#ifndef EXP_PF_H__
#define EXP_PF_H__

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

class ExpResetMcl : public Mcl
{
public: 
	ExpResetMcl(const Pose &p, int num, const Scan &scan,
			const std::shared_ptr<OdomModel> &odom_model,
			const std::shared_ptr<LikelihoodFieldMap> &map,
			double alpha_th, double open_space_th,
			double expansion_radius_position, double expansion_radius_orientation);
	~ExpResetMcl();
	/*

	std::vector<Particle> particles_;
	double alpha_;

	*/
	void sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv);

	/*
	void motionUpdate(double x, double y, double t);

	void initialize(double x, double y, double t);

	void setScan(const sensor_msgs::LaserScan::ConstPtr &msg);
	void meanPose(double &x_mean, double &y_mean, double &t_mean,
			double &x_var, double &y_var, double &t_var,
			double &xy_cov, double &yt_cov, double &tx_cov);

	void simpleReset(void);
	*/

	/*
	static double cos_[(1<<16)];
	static double sin_[(1<<16)];
	*/
private:
	/*
	Pose *last_odom_;
	Pose *prev_odom_;
	*/

	/*
	Scan scan_;
	int processed_seq_;

	double normalizeAngle(double t);
	void resampling(void);
	double normalizeBelief(void);
	void resetWeight(void);

	std::shared_ptr<OdomModel> odom_model_;
	std::shared_ptr<LikelihoodFieldMap> map_;

	*/
	double alpha_threshold_;
	double open_space_threshold_;
	double expansion_radius_position_;
	double expansion_radius_orientation_;

	void expansionReset(void);
};

//double ExpResetMcl::cos_[(1<<16)];
//double ExpResetMcl::sin_[(1<<16)];

}

#endif
