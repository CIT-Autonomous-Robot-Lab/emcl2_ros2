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
    void setSigma(double odom_gnss_sigma, double pf_sigma);
    Eigen::Vector2d odom_gnss_pos_, pf_pos_;
    double pf_x_var_, pf_y_var_;
    double kld();
	  void gnssReset(double alpha, double alpha_th, std::vector<emcl2::Particle> & particles);

      private:
    Eigen::Matrix2d odom_gnss_sigma_, pf_sigma_;
    double det_og_sigma, det_pf_sigma;
    double tr_ogsi_ps;
    double boxMuller(double sigma);
};
}

#endif // EMCL2__OdomGnss_H_