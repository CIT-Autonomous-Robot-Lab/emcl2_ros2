// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "emcl2/LikelihoodFieldMap.h"

#include "emcl2/Pose.h"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <random>

namespace emcl2
{
// Original constructor for OccupancyGrid
LikelihoodFieldMap::LikelihoodFieldMap(
  const nav_msgs::msg::OccupancyGrid & map, double likelihood_range)
{
	width_ = map.info.width;
	height_ = map.info.height;

	origin_x_ = map.info.origin.position.x;
	origin_y_ = map.info.origin.position.y;

	resolution_ = map.info.resolution;

	for (int x = 0; x < width_; x++) {
		likelihoods_.push_back(new uint8_t[height_]);

		for (int y = 0; y < height_; y++) {
			likelihoods_[x][y] = 0;
		}
	}

	for (int x = 0; x < width_; x++) {
		for (int y = 0; y < height_; y++) {
			int v = map.data[x + y * width_];
			if (v > 50) {
				setLikelihood(x, y, likelihood_range);
			} else if (0 <= v && v <= 50) {
				free_cells_.push_back(std::pair<int, int>(x, y));
			}
		}
	}

	//normalize();
}

// New constructor for compressed map data
LikelihoodFieldMap::LikelihoodFieldMap(
  const nav_msgs::msg::MapMetaData & map_info, uint8_t block_size,
  const std::vector<std::vector<int8_t>> & patterns, const std::vector<uint16_t> & block_indices)
{
	width_ = map_info.width;
	height_ = map_info.height;
	resolution_ = map_info.resolution;
	origin_x_ = map_info.origin.position.x;
	origin_y_ = map_info.origin.position.y;

	block_size_ = block_size;
	patterns_ = patterns;
	block_indices_ = block_indices;
	blocks_per_row_ = (width_ + block_size_ - 1) / block_size_;

	// Populate free_cells_ from compressed map data
	free_cells_.clear();
	for (int x = 0; x < width_; ++x) {
		for (int y = 0; y < height_; ++y) {
			int8_t val = getValueFromPattern(
			  getBlockIndex(x, y), x % block_size_, y % block_size_);
			if (val == 0) {
				free_cells_.emplace_back(x, y);
			}
		}
	}
}

LikelihoodFieldMap::~LikelihoodFieldMap()
{
	for (auto & row : likelihoods_) {
		delete[] row;
	}
}

uint8_t LikelihoodFieldMap::likelihood(double x, double y)
{
	// Convert world coordinates to grid coordinates
	int grid_x = static_cast<int>((x - origin_x_) / resolution_);
	int grid_y = static_cast<int>((y - origin_y_) / resolution_);

	// Check if the point is within the map bounds
	if (grid_x < 0 || grid_x >= width_ || grid_y < 0 || grid_y >= height_) {
		return 0;
	}

	// If using compressed map data
	if (!likelihoods_.empty()) {
		return likelihoods_[grid_x][grid_y];
	}

	// Get the value from the compressed map
	int8_t value = getValueFromPattern(
	  getBlockIndex(grid_x, grid_y), grid_x % block_size_, grid_y % block_size_);

	// Return binary value (255 for obstacles, 0 for free space)
	return (value > 0) ? 255 : 0;
}

void LikelihoodFieldMap::drawFreePoses(int num, std::vector<Pose> & result)
{
	std::random_device seed_gen;
	std::mt19937 engine(seed_gen());
	std::uniform_int_distribution<> dist(0, free_cells_.size() - 1);

	result.clear();
	for (int i = 0; i < num; i++) {
		int idx = dist(engine);
		int x = free_cells_[idx].first;
		int y = free_cells_[idx].second;
		result.push_back(
		  Pose(x * resolution_ + origin_x_, y * resolution_ + origin_y_, 0.0));
	}
}

uint16_t LikelihoodFieldMap::getBlockIndex(int grid_x, int grid_y) const
{
	int block_x = grid_x / block_size_;
	int block_y = grid_y / block_size_;
	return block_indices_[block_y * blocks_per_row_ + block_x];
}

int8_t LikelihoodFieldMap::getValueFromPattern(
  uint16_t pattern_index, int block_x, int block_y) const
{
	return patterns_[pattern_index][block_y * block_size_ + block_x];
}

void LikelihoodFieldMap::setLikelihood(int x, int y, double likelihood_range)
{
	int cell_range = static_cast<int>(likelihood_range / resolution_);
	for (int i = -cell_range; i <= cell_range; i++) {
		for (int j = -cell_range; j <= cell_range; j++) {
			int nx = x + i;
			int ny = y + j;
			if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) {
				continue;
			}
			double dist = resolution_ * sqrt(i * i + j * j);
			if (dist > likelihood_range) {
				continue;
			}
			likelihoods_[nx][ny] =
			  static_cast<uint8_t>(255 * (1.0 - dist / likelihood_range));
		}
	}
}

}  // namespace emcl2
