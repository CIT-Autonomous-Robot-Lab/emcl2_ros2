// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "emcl2/GnssReset.h"
#include "cmath"
#include "rclcpp/rclcpp.hpp"

namespace emcl2{
GnssReset::GnssReset()
{
	setSigma(5.0, 1.0);
}

void GnssReset::setSigma(double odom_gnss_sigma, double pf_sigma)
{
    gnss_reset_sigma_ = 1.5;
    odom_gnss_sigma_ << odom_gnss_sigma, 0, 
                        0, odom_gnss_sigma;
    pf_sigma_ << pf_sigma, 0, 
                    0, pf_sigma;
	det_og_sigma = odom_gnss_sigma_.determinant();
	det_pf_sigma = pf_sigma_.determinant();
	tr_ogsi_ps = (odom_gnss_sigma_.transpose() * pf_sigma_).trace();
}

double GnssReset::boxMuller(double sigma)
{
	double r1 = static_cast<double>(rand()) / RAND_MAX;
	double r2 = static_cast<double>(rand()) / RAND_MAX;
	return sigma * (-2.0 * log(r1) * cos(2*M_PI*r2));
}

double GnssReset::kld()
{
    if(isNAN()) return NAN;
	return log10(det_og_sigma / det_pf_sigma) + tr_ogsi_ps + (pf_pos_ - odom_gnss_pos_).transpose() * odom_gnss_sigma_.transpose() * (pf_pos_ - odom_gnss_pos_);
}

void GnssReset::gnssReset(double alpha, double alpha_th, std::vector<emcl2::Particle> & particles)
{
    double beta = alpha < alpha_th ? 1 - alpha / alpha_th : 0.0;
    int particle_num = beta * particles.size();
    RCLCPP_INFO(rclcpp::get_logger("emcl2_node"), "beta: %lf, num of replace particle: %d", beta, particle_num);
    // RCLCPP_INFO(rclcpp::get_logger("emcl2_node"), "(odom_gnss.x, odom_gnss.y) = (%lf, %lf)", odom_gnss_.odom_gnss_x_, odom_gnss_.odom_gnss_y_);
    for(int i=0; i<particle_num; ++i)
    {
        // int index = rand() % particles.size();
        particles[i].p_.x_ = odom_gnss_pos_[0] + boxMuller(gnss_reset_sigma_);
        particles[i].p_.y_ = odom_gnss_pos_[1] + boxMuller(gnss_reset_sigma_);
        particles[i].p_.t_ = ((rand() % 628) - 314) / 100;
        // particles[index].p_.t_ = boxMuller(3.14);
    }
}

bool GnssReset::isNAN()
{
    if(std::isnan(odom_gnss_pos_[0])) return true;
   //  if(odom_gnss_pos_[0] == NAN && odom_gnss_pos_[1] == NAN) return true;
    return false;
}
}
