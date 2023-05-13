//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: BSD-3-Clause

#ifndef OCC_GRID_MAP_H__
#define OCC_GRID_MAP_H__

#include <utility>
#include <vector>

#include "emcl2/Pose.h"
#include "emcl2/Scan.h"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace emcl2
{
class LikelihoodFieldMap
{
      public:
	LikelihoodFieldMap(const nav_msgs::msg::OccupancyGrid & map, double likelihood_range);
	~LikelihoodFieldMap();

	void setLikelihood(int x, int y, double range);
	double likelihood(double x, double y);

	std::vector<double *> likelihoods_;
	int width_;
	int height_;

	double resolution_;
	double origin_x_;
	double origin_y_;

	void drawFreePoses(int num, std::vector<Pose> & result);

      private:
	std::vector<std::pair<int, int> > free_cells_;

	void normalize(void);
};

}  // namespace emcl2

#endif
