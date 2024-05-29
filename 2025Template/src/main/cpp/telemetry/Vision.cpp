
#include "../../include/telemetry/Vision.hpp"

static void Vision::Camera() 
{
    // Get the USB camera from CameraServer
    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();

    // Set the resolution
    camera.SetResolution(640, 480);

    // Get a CvSink. This will capture Mats from the Camera
    cs::CvSink cvSink = frc::CameraServer::GetVideo();

    // Setup a CvSource. This will send images back to the Dashboard
    cs::CvSource outputStream = frc::CameraServer::PutVideo("Rectangle", 640, 480);

    // Mats are very memory expensive. Lets reuse this Mat.
    cv::Mat mat;

    while (true) 
    {
        // Tell the CvSink to grab a frame from the camera and put it in the
        // source mat. If there is an error notify the output.
        if (cvSink.GrabFrame(mat) == 0) 
        {
            // Send the output the error.
            outputStream.NotifyError(cvSink.GetError());

            // skip the rest of the current iteration
            continue;
        }

        // Put a rectangle on the image
        rectangle(mat, cv::Point(100, 100), cv::Point(400, 400), cv::Scalar(255, 255, 255), 5);

        // Give the output stream a new image to display
        outputStream.PutFrame(mat);
    }
}

/// @brief This is the constructor for vision, which doesn't actually need to exist, yeah I'm going to take this out
void Vision::VisionInit() 
{
  std::thread VisionThread(&Vision::Camera);
  VisionThread.detach();
}

void Vision::PoseEstimationPeriodic(Swerve *Gettin_The_ULTIMATE_POSE_ESTIMATOR, bool team)
{
  bool doRejectUpdate = false;

  LimelightHelpers::SetRobotOrientation("limelight", Gettin_The_ULTIMATE_POSE_ESTIMATOR->Ulitmate_Pose_Estimation.GetEstimatedPosition().Rotation().Degrees(), 0, 0, 0, 0, 0);

  LimelightHelpers::PoseEstimate mt2;

  if (team) {
    mt2 = bmt2;
  } else {
    mt2 = rmt2;
  }

  if(std::abs(Gettin_The_ULTIMATE_POSE_ESTIMATOR->navx.GetRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
  {
    doRejectUpdate = true;
  }
  if(mt2.tagCount == 0)
  {
    doRejectUpdate = true;
  }
  if(!doRejectUpdate)
  {
    Gettin_The_ULTIMATE_POSE_ESTIMATOR->Ulitmate_Pose_Estimation.SetVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
    Gettin_The_ULTIMATE_POSE_ESTIMATOR->Ulitmate_Pose_Estimation.AddVisionMeasurement(
      mt2.pose,
      mt2.timestampSeconds
    );
    this->currentPose = mt2.pose;
  }
}