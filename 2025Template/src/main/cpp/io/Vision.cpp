std::make_pair<Transform3d, double> getEstimatedPosition()
{
  frc::Pose3d pose = GetPose();
  table->PutNumber("X", pose.X().value());
  table->PutNumber("Y", pose.Y().value());
  table->PutNumber("Z", pose.Z().value());
  table->PutNumber("Roll", pose.Rotation().X().value());
  table->PutNumber("Pitch", pose.Rotation().Y().value());
  table->PutNumber("Yaw", pose.Rotation().Z().value());
}

#include "../../include/io/Vision.hpp"
}
  return _estimator.Update().first;
frc::Pose3d Vision::GetPose() {