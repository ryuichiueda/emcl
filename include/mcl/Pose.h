/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#ifndef POSE_H__
#define POSE_H__

#include <sstream>

using namespace std;

class Pose
{
public:
	Pose(double x, double y, double t);

	void set(double x, double y, double t);
	void set(const Pose &p);
	string to_s(void);

	void normalizeAngle(void);
	void move(double length, double direction, double rotation,
		  double fw_noise, double rot_noise);

	Pose operator -(const Pose &p) const;
	Pose operator =(const Pose &p);

	bool nearlyZero(void);

	double x_, y_, t_;
};

#endif
