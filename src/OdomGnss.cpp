// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "emcl2/OdomGnss.h"

namespace emcl2{
OdomGnss & OdomGnss::operator=(const OdomGnss & og)
{
    odom_gnss_x_ = og.odom_gnss_x_;
    odom_gnss_y_ = og.odom_gnss_y_;
    return *this;
}

void OdomGnss::update(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    odom_gnss_x_ = msg->pose.pose.position.x;
    odom_gnss_y_ = msg->pose.pose.position.y;
}
}