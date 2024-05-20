#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>

#include "frc/Timer.h"

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>

// header

using namespace frc;

class Vision
{
   photon::PhotonCamera pCamera1{"PhotonCam1"};

  Transform3d robotToCam1 = Transform3d(
    Translation3d(0_m, 0_m, 0_m),  // position of the camera
    Rotation3d(0_rad, 0_rad, 0_rad)
  );

  AprilTagFieldLayout aprilTagFieldLayout = LoadAprilTagLayoutField(AprilTagField::k2024Crescendo);

  photon::PhotonPoseEstimator camPoseEstimator{
    aprilTagFieldLayout, photon::MULTI_TAG_PNP_ON_COPROCESSOR, std::move(pCamera1), robotToCam1};
};

// main file


  auto camPoseEstimate = camPoseEstimator.Update();

  
  double camEstimatedX = camPoseEstimate->estimatedPose.X().value();
  double camEstimatedY = camPoseEstimate->estimatedPose.Y().value();
  double camEstimatedZ = camPoseEstimate->estimatedPose.Z().value();
  double camEstimatedBearing = (camPoseEstimate->estimatedPose.Rotation().Z().value() / Pi * 180);
