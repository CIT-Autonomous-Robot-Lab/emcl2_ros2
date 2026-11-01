// SPDX-FileCopyrightText: 2026 Keita Sekiguchi / nop
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "emcl2/ExpResetMcl2.h"

#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

class InspectableMcl : public emcl2::ExpResetMcl2
{
public:
  using emcl2::ExpResetMcl2::ExpResetMcl2;

  auto scanSequence() const {return scan_.seq_;}
};

int main(int argc, char ** argv)
{
  if (argc != 2) {
    return 2;
  }
  const std::string behavior(argv[1]);
  // Given: the real filter, a real likelihood field and a valid wall return.
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = grid.info.height = 100;
  grid.info.resolution = 0.1;
  grid.info.origin.position.x = grid.info.origin.position.y = -5.0;
  grid.data.resize(10000, 0);
  grid.data[50 * 100 + 60] = 100;
  auto map = std::make_shared<emcl2::LikelihoodFieldMap>(grid, 0.2);
  auto model = std::make_shared<emcl2::OdomModel>(0.01, 0.0001, 0.01, 0.01);
  emcl2::Scan scan{};
  scan.scan_increment_ = 1;
  InspectableMcl filter(
    emcl2::Pose(0, 0, 0), 3, scan, model, map, 0.0, 0.1, 0.2, 1.0, 0.3, false);
  auto message = std::make_shared<sensor_msgs::msg::LaserScan>();
  message->header.stamp.sec = 10;
  message->header.stamp.nanosec = 100000000;
  message->angle_increment = 0.1;
  message->range_min = 0.1;
  message->range_max = 10;
  message->ranges = {1.0};
  filter.setScan(message);
  filter.sensorUpdate(0, 0, 0, false);
  const auto first_sequence = filter.scanSequence();
  const std::array<double, 3> weights{0.2, 0.3, 0.5};
  for (std::size_t i = 0; i < weights.size(); ++i) {
    filter.particles_[i].w_ = weights[i];
  }

  bool passed = true;
  if (behavior == "duplicate") {
    // When: the odometry loop asks again without a new scan.
    filter.sensorUpdate(0, 0, 0, false);
    // Then: the belief is not normalized or resampled a second time.
    for (std::size_t i = 0; i < weights.size(); ++i) {
      passed &= filter.particles_[i].w_ == weights[i];
    }
  } else if (behavior == "subsecond") {
    // When: a distinct acquisition arrives within the same second.
    message->header.stamp.nanosec = 200000000;
    filter.setScan(message);
    // Then: it has a distinct identity for observation freshness.
    passed = filter.scanSequence() != first_sequence;
  } else if (behavior == "resampling") {
    message->header.stamp.nanosec = 200000000;
    filter.setScan(message);
    // When: a new observation resamples the nonuniform belief.
    filter.sensorUpdate(0, 0, 0, false);
    // Then: selection multiplicity, not retained weights, represents belief.
    for (const auto & particle : filter.particles_) {
      passed &= particle.w_ == 1.0 / 3.0;
    }
  } else if (behavior == "covariance") {
    std::array<double, 36> covariance{};
    covariance[0] = covariance[7] = 0.01;
    covariance[35] = 0.12;
    // When: an initial pose explicitly declares uncertain heading and position.
    passed = filter.initialize(0, 0, 0, covariance);
    // Then: the particle support is not a single pose that ignores covariance.
    bool translated = false;
    bool rotated = false;
    for (const auto & particle : filter.particles_) {
      translated |= particle.p_.x_ != 0 || particle.p_.y_ != 0;
      rotated |= particle.p_.t_ != 0;
    }
    passed &= translated && rotated;
  } else if (behavior == "correlated_covariance") {
    std::array<double, 36> covariance{};
    covariance[0] = 0.01;
    covariance[1] = covariance[6] = 0.02;
    covariance[7] = 0.04;
    // When: position uncertainty has a positive-semidefinite correlation.
    passed = filter.initialize(0, 0, 0, covariance);
    // Then: the joint proposal follows that correlation rather than independent axes.
    for (const auto & particle : filter.particles_) {
      passed &= std::abs(particle.p_.y_ - 2 * particle.p_.x_) < 1e-8;
    }
  } else if (behavior == "invalid_covariance") {
    std::array<double, 36> covariance{};
    covariance[0] = -0.01;
    // When: an invalid external covariance reaches initialization.
    passed = !filter.initialize(1, 2, 3, covariance);
    // Then: it is rejected without changing the existing belief.
    for (const auto & particle : filter.particles_) {
      passed &= particle.p_.x_ == 0 && particle.p_.y_ == 0 && particle.p_.t_ == 0;
    }
  } else {
    return 2;
  }
  std::cout << "SCAN_UPDATES behavior=" << behavior << " passed=" << passed << std::endl;
  return passed ? 0 : 1;
}
