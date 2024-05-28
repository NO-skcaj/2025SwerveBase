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

#include "../../include/subsystems/Swerve.hpp"
#include "../../include/telemetry/LimeLightHelper.h" 

using namespace frc;


class Vision {
     public:
          Vision();

          void PoseEstimationPeriodic(Swerve *Gettin_The_ULTIMATE_POSE_ESTIMATOR, bool team);
          
          static void VisionInit();

          frc::Pose2d currentPose;
     private:

          Swerve* m_swerve;
                    
          LimelightHelpers::PoseEstimate bmt2 = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
          LimelightHelpers::PoseEstimate rmt2 = LimelightHelpers::getBotPoseEstimate_wpiRed_MegaTag2("limelight");
};