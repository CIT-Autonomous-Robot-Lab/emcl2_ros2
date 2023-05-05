//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: BSD-3-Clause
//CAUTION: Some lines came from amcl (LGPL). These lines are commented out now. 

#include "emcl/emcl2_node.h"
#include "emcl/Pose.h"

#include "tf2/utils.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/GetMap.h"
#include "std_msgs/Float32.h"

namespace emcl2 {

EMcl2Node::EMcl2Node() : private_nh_("~")
{
	initCommunication();
	initPF();

	private_nh_.param("odom_freq", odom_freq_, 20);

	init_request_ = false;
	simple_reset_request_ = false;
}

EMcl2Node::~EMcl2Node()
{
}

void EMcl2Node::initCommunication(void)
{
	particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2, true);
	pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("mcl_pose", 2, true);
	alpha_pub_ = nh_.advertise<std_msgs::Float32>("alpha", 2, true);
	laser_scan_sub_ = nh_.subscribe("scan", 2, &EMcl2Node::cbScan, this);
	initial_pose_sub_ = nh_.subscribe("initialpose", 2, &EMcl2Node::initialPoseReceived, this);

	global_loc_srv_ = nh_.advertiseService("global_localization", &EMcl2Node::cbSimpleReset, this);

	private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));
	private_nh_.param("footprint_frame_id", footprint_frame_id_, std::string("base_footprint"));
	private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
	private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));

	tfb_.reset(new tf2_ros::TransformBroadcaster());
	tf_.reset(new tf2_ros::Buffer());
	tfl_.reset(new tf2_ros::TransformListener(*tf_));
}

void EMcl2Node::initPF(void)
{
	std::shared_ptr<LikelihoodFieldMap> map = std::move(initMap());
	std::shared_ptr<OdomModel> om = std::move(initOdometry());

	Scan scan;
	private_nh_.param("laser_min_range", scan.range_min_, 0.0);
	private_nh_.param("laser_max_range", scan.range_max_, 100000000.0);
	private_nh_.param("scan_increment", scan.scan_increment_, 1);

	Pose init_pose;
	private_nh_.param("initial_pose_x", init_pose.x_, 0.0);
	private_nh_.param("initial_pose_y", init_pose.y_, 0.0);
	private_nh_.param("initial_pose_a", init_pose.t_, 0.0);

	int num_particles;
	double alpha_th;
	double ex_rad_pos, ex_rad_ori;
	private_nh_.param("num_particles", num_particles, 0);
	private_nh_.param("alpha_threshold", alpha_th, 0.5);
	private_nh_.param("expansion_radius_position", ex_rad_pos, 0.1);
	private_nh_.param("expansion_radius_orientation", ex_rad_ori, 0.2);

	double extraction_rate, range_threshold; bool sensor_reset;
	private_nh_.param("extraction_rate", extraction_rate, 0.1);
	private_nh_.param("range_threshold", range_threshold, 0.1);
	private_nh_.param("sensor_reset", sensor_reset, true);

	pf_.reset(new ExpResetMcl2(init_pose, num_particles, scan, om, map,
				alpha_th, ex_rad_pos, ex_rad_ori,
				extraction_rate, range_threshold, sensor_reset));
}

std::shared_ptr<OdomModel> EMcl2Node::initOdometry(void)
{
	double ff, fr, rf, rr;
	private_nh_.param("odom_fw_dev_per_fw", ff, 0.19);
	private_nh_.param("odom_fw_dev_per_rot", fr, 0.0001);
	private_nh_.param("odom_rot_dev_per_fw", rf, 0.13);
	private_nh_.param("odom_rot_dev_per_rot", rr, 0.2);
	return std::shared_ptr<OdomModel>(new OdomModel(ff, fr, rf, rr));
}

std::shared_ptr<LikelihoodFieldMap> EMcl2Node::initMap(void)
{
	double likelihood_range;
	private_nh_.param("laser_likelihood_max_dist", likelihood_range, 0.2);

	int num;
	private_nh_.param("num_particles", num, 0);

	nav_msgs::GetMap::Request req;
	nav_msgs::GetMap::Response resp;
	ROS_INFO("Requesting the map...");
	while(!ros::service::call("static_map", req, resp)){
		ROS_WARN("Request for map failed; trying again...");
		ros::Duration d(0.5);
		d.sleep();
	}

	return std::shared_ptr<LikelihoodFieldMap>(new LikelihoodFieldMap(resp.map, likelihood_range));
}

void EMcl2Node::cbScan(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    scan_time_stamp_ = msg->header.stamp;
    scan_frame_id_ = msg->header.frame_id;
    pf_->setScan(msg);
}

void EMcl2Node::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
	init_request_ = true;
	init_x_ = msg->pose.pose.position.x;
	init_y_ = msg->pose.pose.position.y;
	init_t_ = tf2::getYaw(msg->pose.pose.orientation);
}

void EMcl2Node::loop(void)
{
	if(init_request_){
		pf_->initialize(init_x_, init_y_, init_t_);
		init_request_ = false;
	}
	else if(simple_reset_request_){
		pf_->simpleReset();
		simple_reset_request_ = false;
	}

	double x, y, t;
	if(not getOdomPose(x, y, t)){
		ROS_INFO("can't get odometry info");
		return;
	}
	pf_->motionUpdate(x, y, t);

	double lx, ly, lt;
	bool inv;
	if(not getLidarPose(lx, ly, lt, inv)){
		ROS_INFO("can't get lidar pose info");
		return;
	}

	/*
	struct timespec ts_start, ts_end;
	clock_gettime(CLOCK_REALTIME, &ts_start);
	*/
	pf_->sensorUpdate(lx, ly, lt, inv);
	/*
	clock_gettime(CLOCK_REALTIME, &ts_end);
	struct tm tm;
	localtime_r( &ts_start.tv_sec, &tm);
	printf("START: %02d.%09ld\n", tm.tm_sec, ts_start.tv_nsec);
	localtime_r( &ts_end.tv_sec, &tm);
	printf("END: %02d.%09ld\n", tm.tm_sec, ts_end.tv_nsec);
	*/

	double x_var, y_var, t_var, xy_cov, yt_cov, tx_cov;
	pf_->meanPose(x, y, t, x_var, y_var, t_var, xy_cov, yt_cov, tx_cov);

	publishOdomFrame(x, y, t);
	publishPose(x, y, t, x_var, y_var, t_var, xy_cov, yt_cov, tx_cov);
	publishParticles();

	std_msgs::Float32 alpha_msg;
	alpha_msg.data = static_cast<float>(pf_->alpha_);
	alpha_pub_.publish(alpha_msg);
}

void EMcl2Node::publishPose(double x, double y, double t,
			double x_dev, double y_dev, double t_dev,
			double xy_cov, double yt_cov, double tx_cov)
{
	geometry_msgs::PoseWithCovarianceStamped p;
	p.header.frame_id = global_frame_id_;
	p.header.stamp = ros::Time::now();
	p.pose.pose.position.x = x;
	p.pose.pose.position.y = y;

	p.pose.covariance[6*0 + 0] = x_dev;
	p.pose.covariance[6*1 + 1] = y_dev;
	p.pose.covariance[6*2 + 2] = t_dev;

	p.pose.covariance[6*0 + 1] = xy_cov;
	p.pose.covariance[6*1 + 0] = xy_cov;
	p.pose.covariance[6*0 + 2] = tx_cov;
	p.pose.covariance[6*2 + 0] = tx_cov;
	p.pose.covariance[6*1 + 2] = yt_cov;
	p.pose.covariance[6*2 + 1] = yt_cov;
	
	tf2::Quaternion q;
	q.setRPY(0, 0, t);
	tf2::convert(q, p.pose.pose.orientation);

	pose_pub_.publish(p);
}

void EMcl2Node::publishOdomFrame(double x, double y, double t)
{
	geometry_msgs::PoseStamped odom_to_map;
	try{
		tf2::Quaternion q;
		q.setRPY(0, 0, t);
		tf2::Transform tmp_tf(q, tf2::Vector3(x, y, 0.0));
				
		geometry_msgs::PoseStamped tmp_tf_stamped;
		tmp_tf_stamped.header.frame_id = footprint_frame_id_;
		tmp_tf_stamped.header.stamp = scan_time_stamp_;
		tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);
		
		tf_->transform(tmp_tf_stamped, odom_to_map, odom_frame_id_);

	}catch(tf2::TransformException){
		ROS_DEBUG("Failed to subtract base to odom transform");
		return;
	}
	tf2::convert(odom_to_map.pose, latest_tf_);
	
	ros::Time transform_expiration = (ros::Time(scan_time_stamp_.toSec() + 0.2));
	geometry_msgs::TransformStamped tmp_tf_stamped;
	tmp_tf_stamped.header.frame_id = global_frame_id_;
	tmp_tf_stamped.header.stamp = transform_expiration;
	tmp_tf_stamped.child_frame_id = odom_frame_id_;
	tf2::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);
	
	tfb_->sendTransform(tmp_tf_stamped);
}

void EMcl2Node::publishParticles(void)
{
	geometry_msgs::PoseArray cloud_msg;
	cloud_msg.header.stamp = ros::Time::now();
	cloud_msg.header.frame_id = global_frame_id_;
	cloud_msg.poses.resize(pf_->particles_.size());

	for(int i=0;i<pf_->particles_.size();i++){		
		cloud_msg.poses[i].position.x = pf_->particles_[i].p_.x_;
		cloud_msg.poses[i].position.y = pf_->particles_[i].p_.y_;
		cloud_msg.poses[i].position.z = 0; 

		tf2::Quaternion q;
		q.setRPY(0, 0, pf_->particles_[i].p_.t_);
		tf2::convert(q, cloud_msg.poses[i].orientation);
	}		
	particlecloud_pub_.publish(cloud_msg);
}

/* came from amcl. This function must be rewritten 
bool EMcl2Node::getOdomPose(double& x, double& y, double& yaw)
{
	geometry_msgs::PoseStamped ident;
	ident.header.frame_id = footprint_frame_id_;
	ident.header.stamp = ros::Time(0);
	tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);
	
	geometry_msgs::PoseStamped odom_pose;
	try{
		this->tf_->transform(ident, odom_pose, odom_frame_id_);
	}catch(tf2::TransformException e){
    		ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
		return false;
	}
	x = odom_pose.pose.position.x;
	y = odom_pose.pose.position.y;
	yaw = tf2::getYaw(odom_pose.pose.orientation);

	return true;
}*/

bool EMcl2Node::getLidarPose(double& x, double& y, double& yaw, bool& inv)
{
	/* This part came from amcl 
	geometry_msgs::PoseStamped ident;
	ident.header.frame_id = scan_frame_id_;
	ident.header.stamp = ros::Time(0);
	tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);
	
	geometry_msgs::PoseStamped lidar_pose;
	try{
		this->tf_->transform(ident, lidar_pose, base_frame_id_);
	}catch(tf2::TransformException e){
    		ROS_WARN("Failed to compute lidar pose, skipping scan (%s)", e.what());
		return false;
	}
	*/
	x = lidar_pose.pose.position.x;
	y = lidar_pose.pose.position.y;

	double roll, pitch;
	tf2::getEulerYPR(lidar_pose.pose.orientation, yaw, pitch, roll);
	inv = (fabs(pitch) > M_PI/2 || fabs(roll) > M_PI/2) ? true : false;

	return true;
}

int EMcl2Node::getOdomFreq(void){
	return odom_freq_;
}

bool EMcl2Node::cbSimpleReset(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	return simple_reset_request_ = true;
}

}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "mcl_node");
	emcl2::EMcl2Node node;

	ros::Rate loop_rate(node.getOdomFreq());
	while (ros::ok()){
		node.loop();
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::spin();

	return 0;
}

