/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#include "mcl/Pose.h"
#include <sstream>
#include <cmath>
using namespace std;

Pose::Pose(double x, double y, double t)
{
	set(x, y, t);
}

void Pose::set(double x, double y, double t)
{
	x_ = x;
	y_ = y;
	t_ = t;
}

void Pose::set(const Pose &p)
{
	x_ = p.x_;
	y_ = p.y_;
	t_ = p.t_;
}

string Pose::to_s(void)
{
	stringstream s;
	s << "x:" << x_ << "\ty:" << y_ << "\tt:" << t_;
	return s.str();
}

void Pose::normalizeAngle(void)
{
	while(t_ > M_PI)
		t_ -= 2*M_PI;
	while(t_ < -M_PI)
		t_ += 2*M_PI;
}

Pose Pose::operator -(const Pose &p) const
{
	Pose ans(x_ - p.x_, y_ - p.y_, t_ - p.t_);
	ans.normalizeAngle();

	return ans;
}

Pose Pose::operator =(const Pose &p)
{
	x_ = p.x_;
       	y_ = p.y_;
	t_ = p.t_;
	return *this;
}

void Pose::move(double length, double direction, double rotation,
		double fw_noise, double rot_noise)
{
	x_ += (length + fw_noise)*cos(direction + rot_noise + t_);
	y_ += (length + fw_noise)*sin(direction + rot_noise + t_);
	t_ += rotation + rot_noise;
	normalizeAngle();
}

bool Pose::nearlyZero(void)
{
	return fabs(x_) < 0.001 and fabs(y_) < 0.001 and fabs(t_) < 0.001;
}
