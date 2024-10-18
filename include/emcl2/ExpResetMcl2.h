// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef EMCL2__EXPRESETMCL2_H_
#define EMCL2__EXPRESETMCL2_H_

#include "emcl2/Mcl.h"
#include "emcl2/GnssUtil.h"

#include <memory>

#include <rclcpp_action/rclcpp_action.hpp>
#include <wall_tracking_msgs/action/wall_tracking.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

using WallTrackingAction = wall_tracking_msgs::action::WallTracking;
using GoalHandleWallTracking = rclcpp_action::ClientGoalHandle<WallTrackingAction>;

namespace emcl2
{
class ExpResetMcl2 : public Mcl
{
      public:
	ExpResetMcl2(
	  const Pose & p, int num, const Scan & scan, const std::shared_ptr<OdomModel> & odom_model,
	  const std::shared_ptr<LikelihoodFieldMap> & map, double alpha_th,
	  double expansion_radius_position, double expansion_radius_orientation,
	  double extraction_rate, double successive_penetration_threshold, bool sensor_reset, 
      const GnssUtil & gnss_utility, bool use_gnss_reset, bool use_wall_tracking, double gnss_reset_var, 
	  double kld_th, double pf_var_th, 
	  rclcpp_action::Client<WallTrackingAction>::SharedPtr wt_client, 
	  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr last_reset_gnss_pos_pub, 
	  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr reset_pose_aft_wt_pub); 
	~ExpResetMcl2();

	void sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv);

	void setGnssPose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
	void setPfPose(double x, double y, double x_var, double y_var);

      private:
	double alpha_threshold_;
	double expansion_radius_position_;
	double expansion_radius_orientation_;

	double extraction_rate_;
	double range_threshold_;
	bool sensor_reset_;
	bool use_gnss_reset_;
    bool use_wall_tracking_;
    bool wall_tracking_start_;
	double gnss_reset_var_;
	double kld_th_, pf_var_th_;
	bool should_gnss_reset_;
	bool open_place_arrived_, pre_open_place_arrived_;
	bool exec_reset_aft_wt_;
	bool gnss_info_rel_is_low_ = true;
	bool likelihood_watch_ = false;
	std::chrono::system_clock::time_point likelihood_watch_timer_start_;

	rclcpp_action::Client<WallTrackingAction>::SharedPtr wt_client_;
	rclcpp_action::Client<WallTrackingAction>::SendGoalOptions send_goal_options_;
	geometry_msgs::msg::PointStamped last_reset_gnss_pos_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr last_reset_gnss_pos_pub_;
	geometry_msgs::msg::PoseArray reset_pose_aft_wt_;
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr reset_pose_aft_wt_pub_;

	void goalResponseCallback(const GoalHandleWallTracking::SharedPtr & goal_handle);
    void feedbackCallback(
        [[maybe_unused]] typename GoalHandleWallTracking::SharedPtr, 
        [[maybe_unused]] const std::shared_ptr<const typename WallTrackingAction::Feedback> feedback);
    void resultCallback(const GoalHandleWallTracking::WrappedResult & result);

	void expansionReset(void);
	double nonPenetrationRate(int skip, LikelihoodFieldMap * map, Scan & scan);

	void resetUseWallTracking(Scan & scan);
	bool tooFar();
	double euclideanDistanceFromLastResetPos();
	void gnssResetWithLLCalc(Scan & scan);
	void expResetWithLLCalc(Scan & scan);
	void gnssResetAndExpReset(Scan & scan);
	void sendWTGoal();
	void updateAndPubResetPoseAftWt();
	void pubLastResetGnssPos();
	GnssUtil gnss_utility_;
	bool executeGnssReset();
};

}  // namespace emcl2

#endif	// EMCL2__EXPRESETMCL2_H_
