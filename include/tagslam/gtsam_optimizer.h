#pragma once
#include <boost/program_options.hpp>

// GTSAM related includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/navigation/MagPoseFactor.h>
#include <gtsam/slam/BetweenFactor.h>

namespace tagslam{
    class GTSAMOptimizer {
    public:
        GTSAMOptimizer(std::map<std::string, float>& isam2_info);
        ~GTSAMOptimizer();
        void setInitialPoseValues(const gtsam::Pose3& prior_pose,
                                const gtsam::noiseModel::Diagonal::shared_ptr& pose_noise_model);
        void setTagsInitialValues(const std::string& tags_filepath, const gtsam::noiseModel::Diagonal::shared_ptr& tags_noise_model);
        void runOptimization(const int& secs, const int& nsecs, const gtsam::Pose3& delta_pose);
        void saveResults(const std::string& body_poses_filepath);
        void reset();
        gtsam::Values result_;
        int correction_count_;
        std::vector<int> tags_id_;
        std::shared_ptr<gtsam::NonlinearFactorGraph> graph_;
        gtsam::NavState prev_state_;
        std::vector<gtsam::Pose3> tags_known_poses_;
        std::map<std::string, float> isam2_info_;
        gtsam::Values initial_values_;
        
    private:
        std::shared_ptr<gtsam::ISAM2> isam2_;
        std::vector<int> stamp_secs_;
        std::vector<int> stamp_nsecs_;
        gtsam::noiseModel::Diagonal::shared_ptr pose_noise_model_;
    };
}