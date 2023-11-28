// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#ifndef EMCL2__OdomGnss_H_
#define EMCL2__OdomGnss_H_

#include <nav_msgs/msg/odometry.hpp>
#include "emcl2/Particle.h"
#include <Eigen/Dense>

namespace emcl2
{
class GnssReset
{
      public:
    GnssReset();
    Eigen::Vector2d gnss_position_, pf_position_;
    double pf_x_var_, pf_y_var_;
    double kld();
	  void gnssReset(double alpha, double alpha_th, std::vector<emcl2::Particle> & particles, double gnss_reset_var);
    bool isNAN();
    double pfRanGaussian(double sigma);
    double boxMuller(double sigma);
    void setVariance(double gnss_var, double pf_var);

      private:
    double gnss_var_, pf_var_;
    double gnss_reset_var_;
};
}

#endif // EMCL2__OdomGnss_H_
