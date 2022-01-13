//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef OCC_GRID_MAP_H__
#define OCC_GRID_MAP_H__

#include <vector>
#include <utility>
#include "emcl/Scan.h"
#include "emcl/Pose.h"
#include "nav_msgs/OccupancyGrid.h"

namespace emcl {

class LikelihoodFieldMap
{
public: 
	LikelihoodFieldMap(const nav_msgs::OccupancyGrid &map, double likelihood_range);
	~LikelihoodFieldMap();

	void setLikelihood(int x, int y, double range);
	double likelihood(double x, double y);

	std::vector<double *> likelihoods_;
	int width_;
	int height_;

	double resolution_;
	double origin_x_;
	double origin_y_;

	void drawFreePoses(int num, std::vector<Pose> &result);
private:
	std::vector<std::pair<int, int> > free_cells_;

	void normalize(void);
};

}

#endif

