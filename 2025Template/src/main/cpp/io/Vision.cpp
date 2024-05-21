
#include "../../include/io/Vision.hpp"


 void Vision::Periodic()
{
  frc::Pose3d pose = GetPose();
  table->PutNumber("X", pose.X().value());
  table->PutNumber("Y", pose.Y().value());
  table->PutNumber("Z", pose.Z().value());
  table->PutNumber("Roll", pose.Rotation().X().value());
  table->PutNumber("Pitch", pose.Rotation().Y().value());
  table->PutNumber("Yaw", pose.Rotation().Z().value());
}

std::make_pair<Transform3d, double> Vision::GetEstimatedPosition() {
  return _estimator.Update();
}