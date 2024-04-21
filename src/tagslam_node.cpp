/**
 * @file graph_optimization
 * @brief Description here
 * @author Jose Cuaran (jrcuaranv@gmail.com)
 */
#include<tagslam/tagslam_node.h>

using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::L;  // Point3 (x,y,z)

using namespace gtsam;
using namespace std;

TagslamNode::TagslamNode(ros::NodeHandle& nh):nh_(nh){
  nh_ = ros::NodeHandle("~");
  getRosParams();
  path_sub_ = nh_.subscribe("/tagpose", 10, &TagslamNode::pathCallback, this);
  factors_sub_ = nh_.subscribe("/projection_factors", 10, &TagslamNode::ProjectionFactorsCallback, this);
  vio_sub_ = nh_.subscribe(vio_topic_, 10, &TagslamNode::vioCallback, this);

  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/cam_pose",10);
  
  optimizer = new tagslam::GTSAMOptimizer(isam2_params_);
  if(optimizer == nullptr){ printf("null optimizer pointer\n");}
  else {printf("optimizer was created\n");}
  reset();
  
}

TagslamNode::~TagslamNode() {
  optimizer->saveResults(output_body_poses_path_);
  delete optimizer;
}

void TagslamNode::getRosParams(){
  std::vector<float> initial_pose_noise;
  std::vector<float> apriltags_noise;
  std::vector<float> relative_pose_noise;
  float cam_noise;
  std::vector<float> body_P_left_cam_pos;
  std::vector<float> body_P_left_cam_quat;
  std::vector<float> body_P_right_cam_pos;
  std::vector<float> body_P_right_cam_quat;
  float relinearizeThreshold;
  float relinearizeSkip;
  double stereo_left_intr_fx;
  double stereo_left_intr_fy;
  double stereo_left_intr_cx;
  double stereo_left_intr_cy;
  double stereo_right_intr_fx;
  double stereo_right_intr_fy;
  double stereo_right_intr_cx;
  double stereo_right_intr_cy;
  double stereo_baseline;

  nh_.getParam("/tags_filepath", tags_filepath_);
  nh_.getParam("/vio_topic", vio_topic_);
  nh_.getParam("/output_body_poses_path", output_body_poses_path_);
  nh_.getParam("/global_frame_id", global_frame_id_);
  nh_.getParam("/body_frame_id", body_frame_id_);
  nh_.getParam("/noise_models/cam_noise", cam_noise);
  nh_.getParam("/noise_models/initial_pose_noise", initial_pose_noise);
  nh_.getParam("/noise_models/apriltags_noise", apriltags_noise);
  nh_.getParam("/noise_models/relative_pose_noise", relative_pose_noise);
  
  nh_.getParam("/stereo_cam/left/body_P_cam_pos", body_P_left_cam_pos);
  nh_.getParam("/stereo_cam/left/body_P_cam_quat", body_P_left_cam_quat);
  nh_.getParam("/stereo_cam/right/body_P_cam_pos", body_P_right_cam_pos);
  nh_.getParam("/stereo_cam/right/body_P_cam_quat", body_P_right_cam_quat);
  
  nh_.getParam("/stereo_cam/baseline", stereo_baseline);
  nh_.getParam("/stereo_cam/left/intrinsics/fx", stereo_left_intr_fx);
  nh_.getParam("/stereo_cam/left/intrinsics/fy", stereo_left_intr_fy);
  nh_.getParam("/stereo_cam/left/intrinsics/cx", stereo_left_intr_cx);
  nh_.getParam("/stereo_cam/left/intrinsics/cy", stereo_left_intr_cy);
  nh_.getParam("/stereo_cam/right/intrinsics/fx", stereo_right_intr_fx);
  nh_.getParam("/stereo_cam/right/intrinsics/fy", stereo_right_intr_fy);
  nh_.getParam("/stereo_cam/right/intrinsics/cx", stereo_right_intr_cx);
  nh_.getParam("/stereo_cam/right/intrinsics/cy", stereo_right_intr_cy);
  
  

  nh_.getParam("/isam2/relinearizeThreshold", relinearizeThreshold);
  nh_.getParam("/isam2/relinearizeSkip", relinearizeSkip);
  nh_.getParam("/isam2/reset_after_time", reset_after_time_);
  

  isam2_params_ = {{"relinearizeThreshold", relinearizeThreshold},
                  {"relinearizeSkip", relinearizeSkip}};

  
  K_stereo_ = boost::shared_ptr<gtsam::Cal3_S2Stereo>(new Cal3_S2Stereo(stereo_left_intr_fx,
                                      stereo_left_intr_fy,
                                      0.0,
                                      stereo_left_intr_cx,
                                      stereo_left_intr_cy,
                                      stereo_baseline));
  K_left_ = boost::shared_ptr<gtsam::Cal3_S2>(new Cal3_S2(stereo_left_intr_fx,
                                      stereo_left_intr_fy,
                                      0.0,
                                      stereo_left_intr_cx,
                                      stereo_left_intr_cy));
  K_right_ = boost::shared_ptr<gtsam::Cal3_S2>(new Cal3_S2(stereo_right_intr_fx,
                                      stereo_right_intr_fy,
                                      0.0,
                                      stereo_right_intr_cx,
                                      stereo_right_intr_cy));
  

  //gtsam quat [qw, qx, qy, qz]
  body_P_left_cam_ = gtsam::Pose3(gtsam::Rot3::Quaternion(body_P_left_cam_quat[3],body_P_left_cam_quat[0],body_P_left_cam_quat[1],body_P_left_cam_quat[2]),
                            gtsam::Point3(body_P_left_cam_pos[0],body_P_left_cam_pos[1],body_P_left_cam_pos[2]));
  
  body_P_right_cam_ = gtsam::Pose3(gtsam::Rot3::Quaternion(body_P_right_cam_quat[3],body_P_right_cam_quat[0],body_P_right_cam_quat[1],body_P_right_cam_quat[2]),
                            gtsam::Point3(body_P_right_cam_pos[0],body_P_right_cam_pos[1],body_P_right_cam_pos[2]));
  
  initial_pose_noise_ = noiseModel::Diagonal::Sigmas((Vector(6) << initial_pose_noise[0],
                                                                    initial_pose_noise[1],
                                                                    initial_pose_noise[2],
                                                                    initial_pose_noise[3],
                                                                    initial_pose_noise[4],
                                                                    initial_pose_noise[5]).finished());  //Initial pose noise model rad,rad,rad,m, m, m
  
  apriltags_noise_ = noiseModel::Diagonal::Sigmas((Vector(3) << apriltags_noise[0],
                                                                apriltags_noise[1],
                                                                apriltags_noise[2]).finished());  //Uncertainty in the known apriltags poses rad,rad,rad,m, m, m
  relative_pose_noise_ = gtsam::noiseModel::Diagonal::Sigmas((Vector(6) << relative_pose_noise[0],
                                                              relative_pose_noise[1],
                                                              relative_pose_noise[2],
                                                              relative_pose_noise[3],
                                                              relative_pose_noise[4],
                                                              relative_pose_noise[5]).finished());  // uncertainty in the observed tag poses rad,rad,rad,m, m, m
  stereo_cam_noise_ = noiseModel::Isotropic::Sigma(3, cam_noise);
  left_cam_noise_ = noiseModel::Isotropic::Sigma(2, cam_noise);
  right_cam_noise_ = noiseModel::Isotropic::Sigma(2, cam_noise);
} //end getRosParams

void TagslamNode::reset(){
  printf("Reinitializing node\n");
  optimizer->setTagsInitialValues(tags_filepath_, apriltags_noise_);
  new_tagpose_msg_ = false;
  new_stereo_factor_ = false;
  new_monocular_factor_ = false;
  new_vio_pose_ = false;
  initialization_complete_ = false; 
  optimization_complete_ = true;
  delta_time_ = 0;
  prev_time_ = ros::Time::now().toSec();
  tags_poses_.clear();
  tags_id_.clear();
}
void TagslamNode::optimizerTasks(){
  while(ros::ok()){
  try{
    current_time_ = ros::Time::now().toSec();

    // reset after some time without vio or tag corrections
    delta_time_ = current_time_ - prev_time_; // time since the last projection (correction) factor
    if (delta_time_ > reset_after_time_){
      std::cout << "Reinitializing optimizer. Waiting for tag observations..." << std::endl;
      optimizer->reset();
      reset();
    }
    // initializing the optimizer when the first tag_pose is recieved
    if (initialization_complete_ == false && new_tagpose_msg_ == true and new_vio_pose_ == true){
        printf("Initializing optimizer\n");
        gtsam::Pose3 initial_pose_cam_tag = tags_poses_[0];
        int tag_id = tags_id_[0];
        gtsam::Pose3 pose_w_tag = optimizer->tags_known_poses_[tag_id];
        gtsam::Pose3 pose_w_cam = pose_w_tag * (initial_pose_cam_tag.inverse()); //prior pose
        gtsam::Pose3 pose_w_body = pose_w_cam * (body_P_left_cam_.inverse());
        optimizer->setInitialPoseValues(pose_w_body, initial_pose_noise_);
        prev_vio_ = vio_poses_.back();

        printf("Initialization complete\n");
        tags_poses_.clear();
        tags_id_.clear();
        initialization_complete_ = true;
        new_tagpose_msg_ = false;
        new_stereo_factor_ = false;
        new_monocular_factor_ = false;
        stereo_factors_.clear();
        monocular_factors_.clear();
        new_vio_pose_ = false;
        vio_poses_.clear();
        
      }
    // optimizing after new projection factors or vio measurements
    if (initialization_complete_==true && optimization_complete_==true && new_vio_pose_ == true){
      optimization_complete_ = false;
      gtsam::Pose3 vio_pose_earliest = vio_poses_.back();
      gtsam::Pose3 delta_pose = prev_vio_.inverse() * vio_pose_earliest;
      addVioFactor(delta_pose);  
      prev_vio_ = vio_poses_.back();
      if (new_stereo_factor_==true){
        for (size_t i=0; i< stereo_factors_.size(); i++){
          gtsam::StereoPoint2 stereo_point = gtsam::StereoPoint2(stereo_factors_[i][1], stereo_factors_[i][2], stereo_factors_[i][3]);
          int tag_id = int(stereo_factors_[i][0]);
          addStereoFactor(stereo_point, tag_id);
        }
      }

      if (new_monocular_factor_==true){
        for (size_t i=0; i<monocular_factors_.size(); i++){
          gtsam::Point2 point = gtsam::Point2(monocular_factors_[i][2], monocular_factors_[i][3]);
          int tag_id = int(monocular_factors_[i][0]);
          int cam_id = int(monocular_factors_[i][1]);
          if (cam_id == 0){
            addGenericProjectionFactorLeftCam(point, tag_id);
          }
          if (cam_id == 1){
            addGenericProjectionFactorRightCam(point, tag_id);
          }
        }
      }
      
    
      optimizer->runOptimization(vio_timestamp_.sec, vio_timestamp_.nsec, delta_pose);
      gtsam::Vector3 gtsam_position = optimizer->prev_state_.pose().translation();
      // gtsam::Quaternion gtsam_quat = optimizer->prev_state_.pose().rotation().toQuaternion();
      std::cout << "time: " << vio_timestamp_.sec << " Current position: x:" << gtsam_position(0) << " y:" << gtsam_position(1) << " z:" << gtsam_position(2) << std::endl;
      tfBroadcaster(optimizer->prev_state_.pose(), vio_timestamp_);
      publishOdom(optimizer->prev_state_.pose(), vio_timestamp_);
      
      
      optimization_complete_=true;
      tags_poses_.clear();
      tags_id_.clear();
      stereo_factors_.clear();
      new_stereo_factor_=false;
      monocular_factors_.clear();
      new_monocular_factor_=false;
      new_vio_pose_ = false;
      vio_poses_.clear();
      prev_time_ = ros::Time::now().toSec();
    }
    
  }// end try
  catch (const std::exception&e){
    std::cerr <<"Exception caught. Optimizer will be reinitialized " << e.what() << std::endl;
    optimizer->reset();
    reset();
    
  } //end catch
  } //end while
} //end OptimizerTasks

void TagslamNode::publishOdom(const gtsam::Pose3& pose, const ros::Time& tstamp){
  nav_msgs::Odometry odom;
  odom.header.frame_id = global_frame_id_;
  odom.header.stamp = tstamp;
  odom.child_frame_id = body_frame_id_;

  gtsam::Vector3 position = pose.translation();
  gtsam::Quaternion quat =  pose.rotation().toQuaternion();
  odom.pose.pose.position.x = position(0);
  odom.pose.pose.position.y = position(1);
  odom.pose.pose.position.z = position(2);
  odom.pose.pose.orientation.x = quat.x();
  odom.pose.pose.orientation.y = quat.y();
  odom.pose.pose.orientation.z = quat.z();
  odom.pose.pose.orientation.w = quat.w();
  odom_pub_.publish(odom);
}
void TagslamNode::tfBroadcaster(const gtsam::Pose3& pose, const ros::Time& tstamp){
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.stamp = tstamp;
  transform_stamped.header.frame_id = global_frame_id_;
  transform_stamped.child_frame_id = body_frame_id_;

  gtsam::Vector3 position = pose.translation();
  gtsam::Quaternion quat =  pose.rotation().toQuaternion();

  transform_stamped.transform.translation.x = position(0);
  transform_stamped.transform.translation.y = position(1);
  transform_stamped.transform.translation.z = position(2);
  transform_stamped.transform.rotation.x = quat.x();
  transform_stamped.transform.rotation.y = quat.y();
  transform_stamped.transform.rotation.z = quat.z();
  transform_stamped.transform.rotation.w = quat.w();

  br.sendTransform(transform_stamped);
}
void TagslamNode::pathCallback(const nav_msgs::Path::ConstPtr& msg){
  // printf("New path message recievied\n");
  int n_poses = msg->poses.size();
  
  for (int i=0; i< n_poses; i++){
    gtsam::Rot3 R = gtsam::Rot3::Quaternion(msg->poses[i].pose.orientation.w,
                                            msg->poses[i].pose.orientation.x,
                                            msg->poses[i].pose.orientation.y,
                                            msg->poses[i].pose.orientation.z);
    gtsam::Point3 t = gtsam::Point3(msg->poses[i].pose.position.x,
                                    msg->poses[i].pose.position.y,
                                    msg->poses[i].pose.position.z);
    tags_poses_.push_back(gtsam::Pose3(R,t));
    tags_id_.push_back(msg->poses[i].header.seq);  
  }
  path_timestamp_ = msg->header.stamp;
  new_tagpose_msg_ = true;
}
void TagslamNode::ProjectionFactorsCallback(const nav_msgs::Path::ConstPtr& msg){
  int n_factors = msg->poses.size(); //factors come in a path msg as poses, but they are not poses
  
  for (int i=0; i< n_factors; i++){
    int factor_type = msg->poses[i].pose.orientation.w;
    std::vector<float> stereo_factor;
    std::vector<float> monocular_factor;
    if (factor_type == 2){ //stereo factor
      stereo_factor.push_back(msg->poses[i].header.seq);//tag_id
      stereo_factor.push_back(msg->poses[i].pose.position.x);//u_left
      stereo_factor.push_back(msg->poses[i].pose.position.y);//u_right
      stereo_factor.push_back(msg->poses[i].pose.position.z);//v
      stereo_factors_.push_back(stereo_factor);
      stereo_factor.clear();
      new_stereo_factor_ = true;
    }
    if (factor_type == 3){ //monocular factor
      monocular_factor.push_back(msg->poses[i].header.seq);//tag_id
      monocular_factor.push_back(msg->poses[i].pose.position.x);//cam_id
      monocular_factor.push_back(msg->poses[i].pose.position.y);//u
      monocular_factor.push_back(msg->poses[i].pose.position.z);//v
      monocular_factors_.push_back(monocular_factor);
      monocular_factor.clear();
      new_monocular_factor_ = true;
    }
  }
  projection_factor_timestamp_ = msg->header.stamp;
  
}

void TagslamNode::vioCallback(const nav_msgs::Odometry::ConstPtr& msg){
  gtsam::Pose3 vio_pose = gtsam::Pose3(gtsam::Rot3::Quaternion(msg->pose.pose.orientation.w,
                                                              msg->pose.pose.orientation.x,
                                                              msg->pose.pose.orientation.y,
                                                              msg->pose.pose.orientation.z),
                                      gtsam::Point3(msg->pose.pose.position.x,
                                                    msg->pose.pose.position.y,
                                                      msg->pose.pose.position.z));
  vio_poses_.push_back(vio_pose);
  vio_timestamp_ = msg->header.stamp;
  new_vio_pose_ = true;
}

void TagslamNode::addStereoFactor(const gtsam::StereoPoint2& stereo_point, const int& tag_id){
  cout << "Adding stereo factor" << endl;
  optimizer->graph_->emplace_shared<GenericStereoFactor<Pose3, Point3> >(
      stereo_point, stereo_cam_noise_, X(optimizer->correction_count_+1), L(tag_id), K_stereo_, body_P_left_cam_);
}

void TagslamNode::addGenericProjectionFactorLeftCam(const gtsam::Point2& point, const int& tag_id){
  optimizer->graph_->emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(
  point, left_cam_noise_, X(optimizer->correction_count_+1), L(tag_id), K_left_, body_P_left_cam_);
}
void TagslamNode::addGenericProjectionFactorRightCam(const gtsam::Point2& point, const int& tag_id){
  optimizer->graph_->emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(
  point, right_cam_noise_, X(optimizer->correction_count_+1), L(tag_id), K_right_, body_P_right_cam_);
}

void TagslamNode::addVioFactor(const gtsam::Pose3& delta_pose){
  optimizer->graph_->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(optimizer->correction_count_), X(optimizer->correction_count_+1), delta_pose, relative_pose_noise_);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "graph_optimization_node");
  ros::NodeHandle nh;
  TagslamNode ros_topics(nh);
  
  std::thread test_thread(&TagslamNode::optimizerTasks, &ros_topics);
  ros::spin();
  
  test_thread.join();
  return 0;
}
