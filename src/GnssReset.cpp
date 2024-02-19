// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "emcl2/GnssReset.h"
#include "cmath"
#include "rclcpp/rclcpp.hpp"

namespace emcl2{
GnssReset::GnssReset()
{
  gnss_var_ = 5.0;
  pf_var_ = 1.0;
  setVariance(gnss_var_, pf_var_);
}

void GnssReset::setVariance(double gnss_var, double pf_var)
{
  gnss_var_ = gnss_var;
  pf_var_ = pf_var;
}

double GnssReset::boxMuller(double sigma)
{
	double r1 = static_cast<double>(rand()) / RAND_MAX;
	double r2 = static_cast<double>(rand()) / RAND_MAX;
	return sigma * log(r1) * cos(2*M_PI*r2);
}

double GnssReset::pfRanGaussian(double sigma)
{
  double x1, x2, w, r;
  do
  {
    do { r = drand48(); } while (r==0.0);
    x1 = 2.0 * r - 1.0;
    do { r = drand48(); } while (r==0.0);
    x2 = 2.0 * r - 1.0;
    w = x1*x1 + x2*x2;
  } while(w > 1.0 || w==0.0);
  return sigma * x2 * sqrt(-2.0*log(w)/w);
}

double GnssReset::kld()
{
    if(isNAN()) return NAN;
    Eigen::Matrix2d gnss_sigma_mx, pf_sigma_mx;
    gnss_sigma_mx << gnss_var_, 0.0, 
                      0.0, gnss_var_;
    pf_sigma_mx << pf_var_, 0.0, 
                    0.0, pf_var_;
    double kld = log(gnss_sigma_mx.determinant() / pf_sigma_mx.determinant());
    kld += (gnss_sigma_mx.inverse() * pf_sigma_mx).trace();
    Eigen::Vector2d diff_pos = pf_position_ - gnss_position_;
    kld += diff_pos.transpose() * gnss_sigma_mx.inverse() * diff_pos;
    kld -= 2.0;
    kld /= 2.0;
    return kld;
}

void GnssReset::gnssReset(double alpha, double alpha_th, std::vector<emcl2::Particle> & particles, double gnss_reset_sigma)
{
    double beta = alpha < alpha_th ? 1 - alpha / alpha_th : 0.0;
    int particle_num = beta * particles.size();
    RCLCPP_INFO(rclcpp::get_logger("emcl2_node"), "beta: %lf, num of replace particle: %d", beta, particle_num);
    // RCLCPP_INFO(rclcpp::get_logger("emcl2_node"), "(odom_gnss.x, odom_gnss.y) = (%lf, %lf)", odom_gnss_.odom_gnss_x_, odom_gnss_.odom_gnss_y_);
    for(int i=0; i<particle_num; ++i)
    {
    //     double length =
		//   2 * (static_cast<double>(rand()) / RAND_MAX - 0.5) * gnss_reset_sigma;
		// double direction = 2 * (static_cast<double>(rand()) / RAND_MAX - 0.5) * M_PI;
        // int index = rand() % particles.size();
        particles[i].p_.x_ = gnss_position_[0] + pfRanGaussian(gnss_reset_sigma);
        particles[i].p_.y_ = gnss_position_[1] + pfRanGaussian(gnss_reset_sigma);
        // particles[i].p_.x_ = gnss_position_[0] + length * cos(direction);
        // particles[i].p_.y_ = gnss_position_[1] + length * sin(direction);
        particles[i].p_.t_ = 2 * (static_cast<double>(rand()) / RAND_MAX - 0.5) * M_PI;
        // particles[i].p_.t_ = pfRanGaussian(3.14);
    }
}

bool GnssReset::isNAN()
{
    if(std::isnan(gnss_position_[0]) || std::isnan(gnss_position_[1])) return true;
    return false;
}
}
