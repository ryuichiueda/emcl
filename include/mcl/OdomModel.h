/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#ifndef ODOM_MODEL_H__
#define ODOM_MODEL_H__

#include <random>

using namespace std;

class OdomModel
{
public:
	OdomModel(double ff, double fr, double rf, double rr);
	void setDev(double length, double angle);
	double drawFwNoise(void);
	double drawRotNoise(void);
private:
	double fw_dev_;
	double rot_dev_;

	double fw_var_per_fw_;
	double fw_var_per_rot_;
	double rot_var_per_fw_;
	double rot_var_per_rot_;

	std::normal_distribution<> std_norm_dist_;
	
	std::random_device seed_gen_;
	std::default_random_engine engine_;
};

#endif
