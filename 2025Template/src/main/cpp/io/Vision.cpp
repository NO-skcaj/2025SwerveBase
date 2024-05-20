#include "../../include/io/Vision.hpp"

std::make_pair<Transform3d, double> getEstimatedPosition()
{
      auto camPoseEstimate = camPoseEstimator.Update();

      Pose2d estimatedPose(
         camPoseEstimate->estimatedPose.X().value()
         camPoseEstimate->estimatedPose.Y().value()
         camPoseEstimate->estimatedPose.Z().value()
         units::degrees_t(camEstimatedBearing = (camPoseEstimate->estimatedPose.Rotation().Z().value() / Pi * 180))
      }
}
