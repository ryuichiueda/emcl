/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#include "mcl/LikelihoodFieldMap.h"
#include <random>
#include <algorithm>
#include "mcl/Pose.h"
using namespace std;

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
		for(int y=0; y<height_; y++){
			int v = map.data[x + y*width_];
			if(v > 50)
				setLikelihood(x, y, likelihood_range);
			else if(0 <= v and v <= 50)
				free_cells_.push_back(pair<int, int>(x,y));
		}

	normalize();
}

LikelihoodFieldMap::~LikelihoodFieldMap()
{
	for(auto &e : likelihoods_)
		delete [] e;
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
	vector<double> weights;
	for(int i=0;i<=cell_num;i++)
		weights.push_back(1.0 - (double)i/cell_num);

	for(int i=-cell_num; i<=cell_num; i++)
		for(int j=-cell_num; j<=cell_num; j++)
			likelihoods_[i+x][j+y] = max(likelihoods_[i+x][j+y], 
			                         min(weights[abs(i)], weights[abs(j)]));
}

void LikelihoodFieldMap::normalize(void)
{
	double maximum = 0.0;
	for(int x=0; x<width_; x++)
		for(int y=0; y<height_; y++)
			maximum = std::max(likelihoods_[x][y], maximum);

	for(int x=0; x<width_; x++)
		for(int y=0; y<height_; y++)
			likelihoods_[x][y] /= maximum;
}

void LikelihoodFieldMap::drawFreePoses(int num, vector<Pose> &result)
{
	random_device seed_gen;
	mt19937 engine{seed_gen()};
	vector<pair<int, int> > chosen_cells;
	
	sample(free_cells_.begin(), free_cells_.end(), back_inserter(chosen_cells), num, engine);

	for(auto &c : chosen_cells){
		Pose p;
		p.x_ = c.first*resolution_ + resolution_*rand()/RAND_MAX + origin_x_;
		p.y_ = c.second*resolution_ + resolution_*rand()/RAND_MAX + origin_y_;
		p.t_ = 2*M_PI*rand()/RAND_MAX - M_PI;
		result.push_back(p);
	}
}
