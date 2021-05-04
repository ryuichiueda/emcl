/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#include "mcl/LikelihoodFieldMap.h"

LikelihoodFieldMap::LikelihoodFieldMap(const nav_msgs::OccupancyGrid &map, double likelihood_range)
{
	width_ = map.info.width;
	height_ = map.info.height;

	origin_x_ = map.info.origin.position.x;
	origin_y_ = map.info.origin.position.y;

	resolution_ = map.info.resolution;

	for(int x=0; x<width_; x++){
		likelihoods_.push_back(new double[height_]);

		for(int y=0; y<height_; y++)
			likelihoods_[x][y] = 0.0;
	}

	for(int x=0; x<width_; x++)
		for(int y=0; y<height_; y++)
			if(map.data[x + y*width_] > 50)
				setLikelihood(x, y, likelihood_range);
}

LikelihoodFieldMap::~LikelihoodFieldMap()
{
	for(auto &c : likelihoods_)
		delete [] c;
}


double LikelihoodFieldMap::likelihood(double x, double y)
{
	int ix = (int)floor((x - origin_x_)/resolution_);
	int iy = (int)floor((y - origin_y_)/resolution_);

	if(ix < 0 or iy < 0 or ix >= width_ or iy >= height_)
		return 0.0;

	return likelihoods_[ix][iy];
}

void LikelihoodFieldMap::setLikelihood(int x, int y, double range)
{
	int cell_num = (int)ceil(range/resolution_);
	std::vector<double> weights;
	for(int i=0;i<=cell_num;i++)
		weights.push_back(1.0 - (double)i/cell_num);

	for(int i=-cell_num; i<=cell_num; i++)
		for(int j=-cell_num; j<=cell_num; j++)
			likelihoods_[i+x][j+y] += std::min(weights[abs(i)], weights[abs(j)]);
}
