//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: BSD-3-Clause
//CAUTION: Some lines came from amcl (LGPL). These lines are commented out now.

#ifndef INTERFACE_EMCL2_H__
#define INTERFACE_EMCL2_H__

// #include <ros/ros.h>

// #include "emcl2/ExpResetMcl2.h"

/* came from amcl (LGPL). But these lines will be the same even if anyone creates. 
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
*/
// #include "geometry_msgs/PoseWithCovarianceStamped.h"
// #include "sensor_msgs/LaserScan.h"
// #include "std_srvs/Empty.h"
#include "tf2/LinearMath/Transform.h"
#include "rclcpp/rclcpp.hpp"

namespace emcl2
{

class EMcl2Node : public rclcpp::Node
{
public:
  EMcl2Node();
  ~EMcl2Node();

  void loop(void);
  int getOdomFreq(void);

private:
  // std::shared_ptr<ExpResetMcl2> pf_;
  /* came from amcl. 
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	ros::Publisher particlecloud_pub_; 
	ros::Publisher pose_pub_;
	*/
  // ros::Publisher alpha_pub_;
  /* came from amcl. 
	ros::Subscriber laser_scan_sub_;
	ros::Subscriber initial_pose_sub_;

	ros::ServiceServer global_loc_srv_;
	*/

  // ros::Time scan_time_stamp_;

  /* came from amcl. 
	std::string footprint_frame_id_;
	std::string global_frame_id_;
	std::string odom_frame_id_;
	std::string scan_frame_id_;
	std::string base_frame_id_;
	*/

  /* came from amcl. 
	std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
	std::shared_ptr<tf2_ros::TransformListener> tfl_;
	std::shared_ptr<tf2_ros::Buffer> tf_;

	tf2::Transform latest_tf_;
	*/

  int odom_freq_;
  bool init_request_;
  bool simple_reset_request_;
  double init_x_, init_y_, init_t_;

  size_t count_;

  void publishPose(
      double x, double y, double t, double x_dev, double y_dev, double t_dev, double xy_cov,
      double yt_cov, double tx_cov);
  void publishOdomFrame(double x, double y, double t);
  void publishParticles(void);
  void sendTf(void);
  bool getOdomPose(double & x, double & y, double & yaw);  //same name is found in amcl
  bool getLidarPose(double & x, double & y, double & yaw, bool & inv);

  void initCommunication(void);
  void initPF(void);
  // std::shared_ptr<LikelihoodFieldMap> initMap(void);
  // std::shared_ptr<OdomModel> initOdometry(void);

  // void cbScan(const sensor_msgs::LaserScan::ConstPtr & msg);
  // bool cbSimpleReset(std_srvs::Empty::Request & req, std_srvs::Empty::Response & res);
  // void initialPoseReceived(
  //     const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg);  //same name is found in amcl
};

}  // namespace emcl2

#endif
