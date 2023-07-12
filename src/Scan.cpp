//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: BSD-3-Clause

#include "emcl/Scan.h"

#include <cmath>

namespace emcl2
{

Scan & Scan::operator=(const Scan & s)
{
	if (this == &s) return *this;

	seq_ = s.seq_;
	scan_increment_ = s.scan_increment_;
	angle_max_ = s.angle_max_;
	angle_min_ = s.angle_min_;
	angle_increment_ = s.angle_increment_;
	range_max_ = s.range_max_;
	range_min_ = s.range_min_;

	// It's not thread safe.
	ranges_.clear();
	copy(s.ranges_.begin(), s.ranges_.end(), back_inserter(ranges_));

	return *this;
}

int Scan::countValidBeams(double * rate)
{
	int ans = 0;
	for (int i = 0; i < ranges_.size(); i += scan_increment_)
		if (valid(ranges_[i])) ans++;

	if (rate != NULL) *rate = (double)ans / ranges_.size() * scan_increment_;

	return ans;
}

bool Scan::valid(double range)
{
	if (std::isnan(range) or std::isinf(range)) return false;

	return range_min_ <= range and range <= range_max_;
}

}  // namespace emcl2
