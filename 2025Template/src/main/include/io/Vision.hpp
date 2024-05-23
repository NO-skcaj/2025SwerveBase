#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>

#include <frc/RobotBase.h>

#include <thread>

#include <cameraserver/CameraServer.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/estimation/VisionEstimation.h>
#include <photon/simulation/VisionSystemSim.h>
#include <photon/simulation/VisionTargetSim.h>
#include <photon/targeting/PhotonPipelineResult.h>

#include <limits>
#include <memory>

using namespace frc;


class Vision {
     public:
          Vision();

          /// @brief the actual Pose3d is a property (GetEstimatedGlobalPose().value_or(frc::Pose3d()).estimatedPose)
          std::optional<photon::EstimatedRobotPose> GetEstimatedGlobalPose();
          
          static void VisionInit();
     private:
          

          photon::PhotonPipelineResult GetLatestResult();

          frc::Field2d& GetSimDebugField();

          void SimPeriodic(frc::Pose2d robotSimPose);

          void ResetSimPose(frc::Pose2d pose);

          Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(frc::Pose2d estimatedPose);

          photon::PhotonPoseEstimator photonEstimator{
          frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo),
          photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
          photon::PhotonCamera{"photonvision"}, frc::Transform3d{frc::Translation3d{0.5_m, 0.0_m, 0.5_m}, frc::Rotation3d{0_rad, 0_rad, 0_rad}}
          };

          std::shared_ptr<photon::PhotonCamera> camera{photonEstimator.GetCamera()};

          /// @brief sim stuff
          std::unique_ptr<photon::VisionSystemSim>     visionSim;
          std::unique_ptr<photon::SimCameraProperties> cameraProp;
          std::shared_ptr<photon::PhotonCameraSim>     cameraSim;

          units::second_t lastEstTimestamp{0_s};
};