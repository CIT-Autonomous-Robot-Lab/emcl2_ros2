// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#ifndef EMCL2__OdomGnss_H_
#define EMCL2__OdomGnss_H_

#include <nav_msgs/msg/odometry.hpp>

namespace emcl2
{
class OdomGnss
{
      public:
    OdomGnss & operator=(const OdomGnss & og);
    double odom_gnss_x_, odom_gnss_y_;
    void update(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
};
}

#endif // EMCL2__OdomGnss_H_