/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#ifndef SCAN_H__
#define SCAN_H__

class Scan
{
public: 
	int seq_;
	int processed_seq_;
	double angle_max_;
	double angle_min_;
	double angle_increment_;
	std::vector<double> ranges_;
};

#endif
