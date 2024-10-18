// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#ifndef EMCL2__GNSSUTIL_H_
#define EMCL2__GNSSUTIL_H_

#include <nav_msgs/msg/odometry.hpp>
#include "emcl2/Particle.h"
#include <Eigen/Dense>

namespace emcl2
{
class GnssUtil
{
      public:
    GnssUtil();
    ~GnssUtil(){};
    Eigen::Vector2d gnss_position_, pf_position_;
    Eigen::Matrix2d gnss_sigma_mx_, pf_sigma_mx_;
    double pf_x_var_, pf_y_var_;
    double kld();
	  void gnssReset(double alpha, double alpha_th, std::vector<emcl2::Particle> & particles);
    bool isNAN();
    double pfRanGaussian(double sigma);
    // double boxMuller(double sigma);
    // void setVariance(double gnss_var);

      private:
    double gnss_var_, pf_var_;
    double gnss_reset_var_;
};
}

#endif // EMCL2__GNSSUTIL_H_
