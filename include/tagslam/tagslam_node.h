#pragma once
#include<tagslam/gtsam_optimizer.h>
#include<ros/ros.h>
#include<tf2_ros/transform_broadcaster.h>
#include<geometry_msgs/TransformStamped.h>
#include<nav_msgs/Path.h>
#include<sensor_msgs/Imu.h>
#include<nav_msgs/Odometry.h>
#include<thread>

class TagslamNode{
  public:
  std::vector<gtsam::Pose3> tags_poses_;
  std::vector<int> tags_id_;
  std::vector<std::vector<float>> stereo_factors_;
  std::vector<std::vector<float>> monocular_factors_;
  std::vector<gtsam::Pose3> vio_poses_;
  gtsam::Pose3 prev_vio_;
  ros::Time path_timestamp_;
  ros::Time projection_factor_timestamp_;
  ros::Time vio_timestamp_;

  bool new_tagpose_msg_;
  bool new_stereo_factor_;
  bool new_monocular_factor_;
  bool new_vio_pose_;
  bool initialization_complete_;
  bool optimization_complete_;
  float delta_time_;
  float reset_after_time_;
  double current_time_;
  double prev_time_;
  gtsam::noiseModel::Diagonal::shared_ptr initial_pose_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr apriltags_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr relative_pose_noise_;
  gtsam::noiseModel::Isotropic::shared_ptr stereo_cam_noise_;
  gtsam::noiseModel::Isotropic::shared_ptr left_cam_noise_;
  gtsam::noiseModel::Isotropic::shared_ptr right_cam_noise_;
  
  gtsam::Pose3 body_P_left_cam_;
  gtsam::Pose3 body_P_right_cam_;
  gtsam::Cal3_S2Stereo::shared_ptr K_stereo_;
  gtsam::Cal3_S2::shared_ptr K_left_;
  gtsam::Cal3_S2::shared_ptr K_right_;
  std::map<std::string, float> isam2_params_;
  std::string tags_filepath_;
  std::string vio_topic_;
  std::string output_topic_;
  std::string output_body_poses_path_; // optimized body poses
  std::string global_frame_id_; 
  std::string body_frame_id_; 
  
  TagslamNode(ros::NodeHandle& nh);
  ~TagslamNode();
  
  void getRosParams();
  void reset();
  void optimizerTasks();
  void publishOdom(const gtsam::Pose3& pose, const ros::Time& tstamp);
  void tfBroadcaster(const gtsam::Pose3& pose, const ros::Time& tstamp);
  void pathCallback(const nav_msgs::Path::ConstPtr& msg);
  void vioCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void ProjectionFactorsCallback(const nav_msgs::Path::ConstPtr& msg);
  void addStereoFactor(const gtsam::StereoPoint2& stereo_point, const int& tag_id);
  void addGenericProjectionFactorLeftCam(const gtsam::Point2& point, const int& tag_id);
  void addGenericProjectionFactorRightCam(const gtsam::Point2& point, const int& tag_id);
  void addVioFactor(const gtsam::Pose3& delta_pose);
  private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber factors_sub_;
    ros::Subscriber vio_sub_;
    ros::Publisher odom_pub_;
    tagslam::GTSAMOptimizer* optimizer;
  
};
