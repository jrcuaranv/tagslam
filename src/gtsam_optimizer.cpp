#include <tagslam/gtsam_optimizer.h>
#include <cstring>
#include <fstream>
#include <iostream>

using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::L;  // Point3 (x,y,z)

namespace tagslam{


static std::string zfill(const std::string& str, size_t width) {
    if (width <= str.length()) {
        return str; // Return original string if width is smaller or equal
    }

    std::string paddedStr = std::string(width - str.length(), '0') + str;
    return paddedStr;
}

static std::shared_ptr<gtsam::ISAM2> makeIsam2(std::map<std::string, float>& isam2_info) {
    printf("Creating a new optimizer\n");
    gtsam::ISAM2Params parameters;
    parameters.evaluateNonlinearError = false; // new, check
    parameters.enablePartialRelinearizationCheck = true; //new, check
    parameters.relinearizeThreshold = isam2_info.at("relinearizeThreshold"); //0.01
    parameters.relinearizeSkip = isam2_info.at("relinearizeSkip");
    std::shared_ptr<gtsam::ISAM2> isam2(new gtsam::ISAM2(parameters));
    if(isam2 == nullptr){ printf("null isam2 pointer\n");}
    else {printf("isam2 pointer was created\n");}
    
    return (isam2);
  }


// constructor
GTSAMOptimizer::GTSAMOptimizer(std::map<std::string, float>& isam2_info):
                                isam2_info_(isam2_info){
    
    std::shared_ptr<gtsam::NonlinearFactorGraph> graph(new gtsam::NonlinearFactorGraph);
    graph_ = graph;
    reset();
}

GTSAMOptimizer::~GTSAMOptimizer() {}

void GTSAMOptimizer::reset(){
    isam2_ = makeIsam2(isam2_info_);
    graph_->resize(0);
    correction_count_ = 0;
    initial_values_.clear();
    result_.clear();
    tags_id_.clear();
    tags_known_poses_.clear();
}
void GTSAMOptimizer::setInitialPoseValues(const gtsam::Pose3& prior_pose,
                                const gtsam::noiseModel::Diagonal::shared_ptr& pose_noise_model){
    pose_noise_model_ = pose_noise_model;
    initial_values_.insert(X(correction_count_), prior_pose);
    
    graph_->addPrior(X(correction_count_), prior_pose, pose_noise_model_);
    gtsam::NavState prev_state(prior_pose, gtsam::Vector3(0,0,0));
    prev_state_ = prev_state;
}

void GTSAMOptimizer::setTagsInitialValues(const std::string& tags_filepath, const gtsam::noiseModel::Diagonal::shared_ptr& tags_noise_model){
    std::ifstream apriltags_file(tags_filepath);
    printf("Setting initial tags values\n");
    if (!apriltags_file.is_open()) {
        std::cout << "Failed to open tags file: " << tags_filepath << std::endl;
    }
    std::string line;
    std::vector<float> tags_x;
    std::vector<float> tags_y;
    std::vector<float> tags_z;
    std::vector<float> tags_rx;
    std::vector<float> tags_ry;
    std::vector<float> tags_rz;
    for (int i=0; i<200; i++){
        tags_known_poses_.push_back(gtsam::Pose3(gtsam::Rot3::Rodrigues(0, 0, 0), gtsam::Point3(0, 0, 0)));

    }
    while (std::getline(apriltags_file, line)) {
        std::string item;
        std::istringstream iss(line);
        std::vector<float> tag;
        while (std::getline(iss, item, ',')) {
            tag.push_back(std::stof(item.c_str()));
        }
        tags_id_.push_back(std::floor(tag[0]));
        tags_x.push_back(tag[1]);
        tags_y.push_back(tag[2]);
        tags_z.push_back(tag[3]);
        tags_rx.push_back(tag[4]);
        tags_ry.push_back(tag[5]);
        tags_rz.push_back(tag[6]);
        gtsam::Pose3 tag_pose = gtsam::Pose3(gtsam::Rot3::Rodrigues(tag[4], tag[5], tag[6]), gtsam::Point3(tag[1], tag[2], tag[3]));
        tags_known_poses_[int(tag[0])] = tag_pose;
        
    }
    apriltags_file.close();
    for (size_t i = 0; i < tags_id_.size(); i++){
        initial_values_.insert(L(tags_id_[i]), gtsam::Point3(tags_x[i], tags_y[i], tags_z[i]));
        }
    // adding priors with noise for all apriltags
    for (size_t i = 0; i < tags_id_.size(); i++){
        graph_->addPrior(L(std::floor(tags_id_[i])),
                gtsam::Point3(tags_x[i], tags_y[i], tags_z[i]), tags_noise_model);
        }
    printf("Setting initial tags values done\n");
} //end setTagInitialValues()

void GTSAMOptimizer::runOptimization(const int& secs, const int& nsecs, const gtsam::Pose3& delta_pose){
    correction_count_++;
    // using delta_pose from odometry to initialize current pose
    initial_values_.insert(X(correction_count_), prev_state_.pose()*delta_pose);
    isam2_->update(*graph_, initial_values_);
    isam2_->update();
    result_ = isam2_->calculateEstimate();

    // reset the graph
    graph_->resize(0);
    initial_values_.clear();
    
    prev_state_ = gtsam::NavState(result_.at<gtsam::Pose3>(X(correction_count_)), gtsam::Vector3(0,0,0));
    stamp_secs_.push_back(secs);
    stamp_nsecs_.push_back(nsecs);
}
void GTSAMOptimizer::saveResults(const std::string& body_poses_filepath){
    
    FILE* body_poses_file = fopen(body_poses_filepath.c_str(), "w+");
    //"timestamp, x(m),y(m),z(m),qx,qy,qz,qw\n");

    for (size_t i = 1; i< stamp_secs_.size(); i++) // skiping index 0, as we dont have the corresponding timestamp
      {
        
        // Getting the body poses estimates after optimization    
        gtsam::Vector3 position = result_.at<gtsam::Pose3>(X(i)).translation();
        gtsam::Quaternion quat =  result_.at<gtsam::Pose3>(X(i)).rotation().toQuaternion();
        std::string nsecs_str = std::to_string(stamp_nsecs_[i]);
        std::string nsecs_padded = zfill(nsecs_str, 9); // just to fix a bug when writing nsecs as integer, sometimes leads to wrong values
        fprintf(body_poses_file, "%i.%s %f %f %f %f %f %f %f\n",
              stamp_secs_[i], nsecs_padded.c_str(), position(0), position(1),
              position(2), quat.x(), quat.y(), quat.z(),
              quat.w());
        // Getting the apriltags' poses estimates after optimization    

      }
      fclose(body_poses_file);
      
    std::cout << "Results have been saved\n\n";
    } // end save results

} // end tagslam namespace


