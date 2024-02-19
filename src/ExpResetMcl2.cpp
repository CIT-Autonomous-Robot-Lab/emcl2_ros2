// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "emcl2/ExpResetMcl2.h"

#include <rclcpp/rclcpp.hpp>

#include <stdlib.h>

#include <cmath>
#include <iostream>

namespace emcl2
{
ExpResetMcl2::ExpResetMcl2(
  const Pose & p, int num, const Scan & scan, const std::shared_ptr<OdomModel> & odom_model,
  const std::shared_ptr<LikelihoodFieldMap> & map, double alpha_th,
  double expansion_radius_position, double expansion_radius_orientation, double extraction_rate,
  double range_threshold, bool sensor_reset, 
  const GnssReset & odom_gnss, bool gnss_reset, bool wall_tracking_flg, double gnss_reset_var, 
  double kld_th, double pf_var_th)
: Mcl::Mcl(p, num, scan, odom_model, map, odom_gnss),
  alpha_threshold_(alpha_th),
  expansion_radius_position_(expansion_radius_position),
  expansion_radius_orientation_(expansion_radius_orientation),
  extraction_rate_(extraction_rate),
  range_threshold_(range_threshold),
  sensor_reset_(sensor_reset), 
  gnss_reset_(gnss_reset), 
  wall_tracking_flg_(wall_tracking_flg),
  gnss_reset_var_(gnss_reset_var), 
  kld_th_(kld_th), 
  pf_var_th_(pf_var_th)
{
	RCLCPP_INFO(rclcpp::get_logger("emcl2_node"), 
	"gnss_reset: %d, wall_tracking_flg: %d, sqrt(gnss_reset_var): %lf, kld_th: %lf, pf_var_th: %lf", 
	gnss_reset_, wall_tracking_flg_, sqrt(gnss_reset_var_), kld_th_, pf_var_th_);
	wall_tracking_start_ = false;
	wall_tracking_cancel_ = false;
}

ExpResetMcl2::~ExpResetMcl2() {}

void ExpResetMcl2::sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv)
{
	Scan scan;
	scan = scan_;

	scan.lidar_pose_x_ = lidar_x;
	scan.lidar_pose_y_ = lidar_y;
	scan.lidar_pose_yaw_ = lidar_t;

	double origin = inv ? scan.angle_max_ : scan.angle_min_;
	int sgn = inv ? -1 : 1;
	for(size_t i = 0; i < scan.ranges_.size() ; i++) {
		scan.directions_16bit_.push_back(Pose::get16bitRepresentation(
			origin + sgn * i * scan.angle_increment_));
	}

	double valid_pct = 0.0;
	int valid_beams = scan.countValidBeams(&valid_pct);
	if (valid_beams == 0) {
		return;
	}

	for (auto & p : particles_) {
		p.w_ *= p.likelihood(map_.get(), scan);
	}

	// RCLCPP_INFO(rclcpp::get_logger("emcl2_node"), "N0.0 particle weight is %lf. Sum weight is %lf.", particles_[0].w_, sum_w);
	alpha_ = nonPenetrationRate(static_cast<int>(particles_.size() * extraction_rate_), map_.get(), scan);
	// RCLCPP_INFO(rclcpp::get_logger("emcl2_node"), "alpha: %lf", alpha_);

	if (alpha_ < alpha_threshold_) {
		if(gnss_reset_ && wall_tracking_flg_) wall_tracking_start_ = true;
		if(!wall_tracking_flg_ || open_place_arrived_){
			double kld = odom_gnss_.kld();
			RCLCPP_INFO(rclcpp::get_logger("emcl2_node"), 
						"kld: %lf / kld_th: %lf, x_var: %lf, y_var: %lf", 
						kld, kld_th_, odom_gnss_.pf_x_var_, odom_gnss_.pf_y_var_);
			bool should_exp_reset = (kld < kld_th_ || (odom_gnss_.pf_x_var_ < pf_var_th_ && odom_gnss_.pf_y_var_ < pf_var_th_) && !should_gnss_reset_);
			RCLCPP_INFO(rclcpp::get_logger("emcl2_node"), "should exp reset: %d", should_exp_reset);
			if(should_exp_reset || !gnss_reset_){
				RCLCPP_INFO(rclcpp::get_logger("emcl2_node"), "EXPANSION RESET");
				expansionReset();
			} else {
				if(!odom_gnss_.isNAN()){
					RCLCPP_INFO(rclcpp::get_logger("emcl2_node"), "GNSS RESET");
					odom_gnss_.setVariance(gnss_reset_var_, expansion_radius_position_*expansion_radius_position_);
					odom_gnss_.gnssReset(alpha_, alpha_threshold_, particles_, sqrt(gnss_reset_var_));
					should_gnss_reset_ = false;
				}
			}
			for (auto & p : particles_) {
				p.w_ *= p.likelihood(map_.get(), scan);
			}
		}
	}
	if(alpha_ >= alpha_threshold_ && open_place_arrived_ && wall_tracking_start_){
		RCLCPP_INFO(rclcpp::get_logger("ExpResetMcl2"), "set wall_tracking_cancel true");
		wall_tracking_cancel_ = true;
	}
	

	if (normalizeBelief() > 0.000001) {
		resampling();
	} else {
		resetWeight();
	}

	processed_seq_ = scan_.seq_;
}

double ExpResetMcl2::nonPenetrationRate(int skip, LikelihoodFieldMap * map, Scan & scan)
{
	static uint16_t shift = 0;
	int counter = 0;
	int penetrating = 0;
	for (size_t i = shift % skip; i < particles_.size(); i += skip) {
		counter++;
		if (particles_[i].wallConflict(map, scan, range_threshold_, sensor_reset_)) {
			penetrating++;
		}
	}
	shift++;

	// std::cout << penetrating << " " << counter << std::endl;
	return static_cast<double>((counter - penetrating)) / counter;
}

void ExpResetMcl2::expansionReset(void)
{
	for (auto & p : particles_) {
		double length =
		  2 * (static_cast<double>(rand()) / RAND_MAX - 0.5) * expansion_radius_position_;
		double direction = 2 * (static_cast<double>(rand()) / RAND_MAX - 0.5) * M_PI;

		p.p_.x_ += length * cos(direction);
		p.p_.y_ += length * sin(direction);
		p.p_.t_ += 2 * (static_cast<double>(rand()) / RAND_MAX - 0.5) *
			   expansion_radius_orientation_;
		p.w_ = 1.0 / particles_.size();
	}
}

bool ExpResetMcl2::getWallTrackingStartSgn(){return wall_tracking_start_;}
void ExpResetMcl2::setWallTrackingStartSgn(bool sgn){wall_tracking_start_ = sgn;}
bool ExpResetMcl2::getShouldGnssReset(){
	if(pre_open_place_arrived_ == open_place_arrived_) return false;
	return true;
}
void ExpResetMcl2::setShouldGnssReset(bool sgn){should_gnss_reset_ = sgn;}
void ExpResetMcl2::setOpenPlaceArrived(bool sgn){
	pre_open_place_arrived_ = open_place_arrived_;
	open_place_arrived_ = sgn;
}
bool ExpResetMcl2::getWallTrackingCancelSgn(){return wall_tracking_cancel_;}
void ExpResetMcl2::setWallTrackingCancelSgn(bool sgn){wall_tracking_cancel_ = sgn;};

}  // namespace emcl2
