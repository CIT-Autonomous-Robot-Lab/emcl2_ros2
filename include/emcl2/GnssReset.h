// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#ifndef EMCL2__OdomGnss_H_
#define EMCL2__OdomGnss_H_

#include <nav_msgs/msg/odometry.hpp>
#include "emcl2/Particle.h"

namespace emcl2
{
class GnssReset
{
      public:
    GnssReset & operator=(const GnssReset & og);
    double odom_gnss_x_, odom_gnss_y_, pf_x_, pf_y_;
	  void gnssReset(double alpha, double alpha_th, std::vector<emcl2::Particle> & particles);

      private:
    double kld();
    double boxMuller(double sigma);
};
}

#endif // EMCL2__OdomGnss_H_