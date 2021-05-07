/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#ifndef SCAN_H__
#define SCAN_H__

#include <iostream>
#include <cmath>
using namespace std;

class Scan
{
public: 
	int seq_;
	double angle_max_;
	double angle_min_;
	double angle_increment_;
	double range_max_;
	double range_min_;

	std::vector<double> ranges_;

	Scan& operator=(const Scan &s)
	{
		if(this == &s)
			return *this;

		seq_ = s.seq_;
		angle_max_ = s.angle_max_;
		angle_min_ = s.angle_min_;
		angle_increment_ = s.angle_increment_;
		range_max_ = s.range_max_;
		range_min_ = s.range_min_;

		// It's not thread safe.
		ranges_.clear();
		copy(s.ranges_.begin(), s.ranges_.end(), back_inserter(ranges_) );

		return *this;
	}

	int countValidBeams(void)
	{
		int ans = 0;
		for(auto r : ranges_)
			if(valid(r))
				ans++;

		return ans;
	}

	bool valid(double range)
	{
		if( isnan(range) or isinf(range) )
			return false;

		return range_min_ <= range and range <= range_max_;
	}
};

#endif
