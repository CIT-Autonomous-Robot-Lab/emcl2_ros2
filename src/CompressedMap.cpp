// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: BSD-3-Clause

#include "emcl2/CompressedMap.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <rclcpp/rclcpp.hpp>
namespace emcl2
{
CompressedMap::CompressedMap(
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

uint8_t CompressedMap::getValue(double x, double y) const
{
	int ix = static_cast<int>(floor((x - origin_x_) / resolution_));
	int iy = static_cast<int>(floor((y - origin_y_) / resolution_));

	if (ix < 0 || iy < 0 || ix >= width_ || iy >= height_) {
		return 0;
	}

	int8_t value =
	  getValueFromPattern(getBlockIndex(ix, iy), ix % block_size_, iy % block_size_);
	// RCLCPP_INFO(rclcpp::get_logger("compressed_map"), "value: %d", value);
	return (value > 0) ? 255 : 0;
}

void CompressedMap::drawFreePoses(int num, std::vector<Pose> & result) const
{
	std::random_device seed_gen;
	std::mt19937 engine{seed_gen()};
	std::vector<std::pair<int, int>> chosen_cells;

	sample(free_cells_.begin(), free_cells_.end(), back_inserter(chosen_cells), num, engine);

	for (auto & c : chosen_cells) {
		Pose p;
		p.x_ = c.first * resolution_ + resolution_ * rand() / RAND_MAX + origin_x_;
		p.y_ = c.second * resolution_ + resolution_ * rand() / RAND_MAX + origin_y_;
		p.t_ = 2 * M_PI * rand() / RAND_MAX - M_PI;
		result.push_back(p);
	}
}

uint16_t CompressedMap::getBlockIndex(int grid_x, int grid_y) const
{
	int block_x = grid_x / block_size_;
	int block_y = grid_y / block_size_;
	return block_indices_[block_y * blocks_per_row_ + block_x];
}

int8_t CompressedMap::getValueFromPattern(uint16_t pattern_index, int block_x, int block_y) const
{
	return patterns_[pattern_index][block_y * block_size_ + block_x];
}

uint8_t CompressedMap::likelihood(double x, double y) const
{
	// 尤度を直接計算する（尤度場を生成せずに）
	return getValue(x, y);
}

}  // namespace emcl2