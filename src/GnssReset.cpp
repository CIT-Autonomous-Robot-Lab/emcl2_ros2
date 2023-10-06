// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "emcl2/GnssReset.h"
#include "cmath"
#include "rclcpp/rclcpp.hpp"

namespace emcl2{
GnssReset & GnssReset::operator=(const GnssReset & og)
{
    odom_gnss_x_ = og.odom_gnss_x_;
    odom_gnss_y_ = og.odom_gnss_y_;
    return *this;
}

double GnssReset::boxMuller(double sigma)
{
	double r1 = static_cast<double>(rand()) / RAND_MAX;
	double r2 = static_cast<double>(rand()) / RAND_MAX;
	return sigma * (-2.0 * log(r1) * cos(2*M_PI*r2));
}

double GnssReset::kld()
{
	
}

void GnssReset::gnssReset(double alpha, double alpha_th, std::vector<emcl2::Particle> & particles)
{
	double beta = alpha < alpha_th ? 1 - alpha / alpha_th : 0.0;
	int particle_num = beta * particles.size();
	// RCLCPP_INFO(rclcpp::get_logger("emcl2_node"), "beta: %lf, num of replace particle: %d", beta, particle_num);
	// RCLCPP_INFO(rclcpp::get_logger("emcl2_node"), "(odom_gnss.x, odom_gnss.y) = (%lf, %lf)", odom_gnss_.odom_gnss_x_, odom_gnss_.odom_gnss_y_);
	for(int i=0; i<particle_num; ++i)
	{
		int index = rand() % particles.size();
		double sigma = 0.1;
		particles[index].p_.x_ = odom_gnss_x_ + boxMuller(sigma);
		particles[index].p_.y_ = odom_gnss_y_ + boxMuller(sigma);
		particles[index].p_.t_ = (rand() % 314) / 100;
	}
}
}