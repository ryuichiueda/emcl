/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#ifndef SCAN_H__
#define SCAN_H__

#include <vector>

namespace emcl {

class Scan
{
public: 
	int seq_;
	int scan_increment_;
	double angle_max_;
	double angle_min_;
	double angle_increment_;
	double range_max_;
	double range_min_;

	std::vector<double> ranges_;

	Scan& operator=(const Scan &s);
	int countValidBeams(double *rate = NULL);
	bool valid(double range);
};

}

#endif
