// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: BSD-3-Clause

#ifndef EMCL2_COMPRESSED_MAP_H
#define EMCL2_COMPRESSED_MAP_H

#include "emcl2/Pose.h"

#include <nav_msgs/msg/map_meta_data.hpp>

#include <vector>

namespace emcl2
{
class CompressedMap
{
      public:
	CompressedMap(
	  const nav_msgs::msg::MapMetaData & map_info, uint8_t block_size,
	  const std::vector<std::vector<int8_t>> & patterns,
	  const std::vector<uint16_t> & block_indices);
	~CompressedMap() = default;

	uint8_t getValue(double x, double y) const;
	void drawFreePoses(int num, std::vector<Pose> & result) const;

	// 尤度計算用メソッドを追加
	uint8_t likelihood(double x, double y) const;

	// アクセスメソッドを追加
	double getResolution() const { return resolution_; }
	int getWidth() const { return width_; }
	int getHeight() const { return height_; }
	double getOriginX() const { return origin_x_; }
	double getOriginY() const { return origin_y_; }

      private:
	uint16_t getBlockIndex(int grid_x, int grid_y) const;
	int8_t getValueFromPattern(uint16_t pattern_index, int block_x, int block_y) const;

	int width_;
	int height_;
	double resolution_;
	double origin_x_;
	double origin_y_;

	uint8_t block_size_;
	std::vector<std::vector<int8_t>> patterns_;
	std::vector<uint16_t> block_indices_;
	int blocks_per_row_;
	std::vector<std::pair<int, int>> free_cells_;
};

}  // namespace emcl2

#endif	// EMCL2_COMPRESSED_MAP_H