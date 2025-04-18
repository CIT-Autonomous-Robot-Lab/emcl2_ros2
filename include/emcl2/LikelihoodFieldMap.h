// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef EMCL2__LIKELIHOODFIELDMAP_H_
#define EMCL2__LIKELIHOODFIELDMAP_H_

#include "emcl2/Pose.h"
#include "emcl2/Scan.h"

#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

namespace emcl2
{
class LikelihoodFieldMap
{
      public:
	// Original constructor for OccupancyGrid
	LikelihoodFieldMap(const nav_msgs::msg::OccupancyGrid & map, double likelihood_range);

	// New constructor for compressed map data
	LikelihoodFieldMap(
	  const nav_msgs::msg::MapMetaData & map_info, uint8_t block_size,
	  const std::vector<std::vector<int8_t>> & patterns,
	  const std::vector<uint16_t> & block_indices);

	~LikelihoodFieldMap();

	uint8_t likelihood(double x, double y);

	int width_;
	int height_;

	double resolution_;
	double origin_x_;
	double origin_y_;

	void drawFreePoses(int num, std::vector<Pose> & result);

      private:
	std::vector<std::pair<int, int>> free_cells_;
	std::vector<uint8_t *> likelihoods_;

	// Members for compressed map data
	uint8_t block_size_;
	std::vector<std::vector<int8_t>> patterns_;
	std::vector<uint16_t> block_indices_;
	uint32_t blocks_per_row_;  // Number of blocks per row

	// Helper methods for compressed map
	inline uint16_t getBlockIndex(int grid_x, int grid_y) const;
	inline int8_t getValueFromPattern(uint16_t pattern_index, int block_x, int block_y) const;

	// Helper method for likelihood field
	void setLikelihood(int x, int y, double likelihood_range);
};

}  // namespace emcl2

#endif	// EMCL2__LIKELIHOODFIELDMAP_H_
