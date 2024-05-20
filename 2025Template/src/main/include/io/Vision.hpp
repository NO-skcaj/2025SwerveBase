#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>

#include "frc/Timer.h"

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>

using namespace frc;

class Vision
{
   private:
      // Driver Camera
      photon::PhotonCamera DriverCam("DriverCamera");
      // Camera is mounted facing forward, half a meter forward of center, half a
      // meter up from center.
      Transform3d robotToDriverCam(Translation3d(0.5_m, 0_m, 0.5_m),
                                        Rotation3d(0_rad, 0_rad, 0_rad));
      
      // ... Add other cameras here
      // we likely wont use any other cameras for pose estimation
   
      // Assemble the list of cameras & mount locations
      std::vector<
          std::pair<photon::PhotonCamera, Transform3d> >
          cameras;
      cameras.push_back(std::make_pair(DriverCam, robotToDriverCam));
      
      photon::PhotonPoseEstimator camPoseEstimator{
       aprilTagFieldLayout, photon::MULTI_TAG_PNP_ON_COPROCESSOR, std::move(cameras.at(0).first), cameras.at(0).second };
   public:
      /// @brief Gets the position from the camera looking at the apriltags, and the timestamp as a double
      std::make_pair<Transform3d, double> GetEstimatedPosition();
};
