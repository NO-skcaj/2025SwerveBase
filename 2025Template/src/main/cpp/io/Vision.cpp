
#include "../../include/io/Vision.hpp"

void Vision::VisionInit() 
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

Vision::Vision() 
{
  std::thread VisionThread(&VisionInit);
  VisionThread.detach();

  // photonEstimator.SetMultiTagFallbackStrategy(
  //     photon::PoseStrategy::LOWEST_AMBIGUITY);

  // if (frc::RobotBase::IsSimulation()) {
  //   visionSim = std::make_unique<photon::VisionSystemSim>("main");

  //   visionSim->AddAprilTags(constants::Vision::kTagLayout);

  //   cameraProp = std::make_unique<photon::SimCameraProperties>();

  //   cameraProp->SetCalibration(960, 720, frc::Rotation2d{90_deg});
  //   cameraProp->SetCalibError(.35, .10);
  //   cameraProp->SetFPS(15_Hz);
  //   cameraProp->SetAvgLatency(50_ms);
  //   cameraProp->SetLatencyStdDev(15_ms);

  //   cameraSim = std::make_shared<photon::PhotonCameraSim>(camera.get(),
  //                                                         *cameraProp.get());

  //   visionSim->AddCamera(cameraSim.get(), constants::Vision::kRobotToCam);
  //   cameraSim->EnableDrawWireframe(true);
  // }
}

photon::PhotonPipelineResult Vision::GetLatestResult() 
{
  return camera->GetLatestResult();
}

std::optional<photon::EstimatedRobotPose> Vision::GetEstimatedGlobalPose() 
{
  auto visionEst = photonEstimator.Update();
  units::second_t latestTimestamp = camera->GetLatestResult().GetTimestamp();
  bool newResult =
      units::math::abs(latestTimestamp - lastEstTimestamp) > 1e-5_s;
  if (frc::RobotBase::IsSimulation()) {
    if (visionEst.has_value()) {
      GetSimDebugField()
          .GetObject("VisionEstimation")
          ->SetPose(visionEst.value().estimatedPose.ToPose2d());
    } else {
      if (newResult) {
        GetSimDebugField().GetObject("VisionEstimation")->SetPoses({});
      }
    }
  }
  if (newResult) {
    lastEstTimestamp = latestTimestamp;
  }
  return visionEst;
}

Eigen::Matrix<double, 3, 1> Vision::GetEstimationStdDevs(frc::Pose2d estimatedPose) 
{
  Eigen::Matrix<double, 3, 1> estStdDevs = Eigen::Matrix<double, 3, 1>{4, 4, 8};
  auto targets = GetLatestResult().GetTargets();
  int numTags = 0;
  units::meter_t avgDist = 0_m;
  for (const auto& tgt : targets) {
    auto tagPose =
        photonEstimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
    if (tagPose.has_value()) {
      numTags++;
      avgDist += tagPose.value().ToPose2d().Translation().Distance(
          estimatedPose.Translation());
    }
  }
  if (numTags == 0) {
    return estStdDevs;
  }
  avgDist /= numTags;
  if (numTags > 1) {
    estStdDevs = Eigen::Matrix<double, 3, 1>{0.5, 0.5, 1};
  }
  if (numTags == 1 && avgDist > 4_m) {
    estStdDevs = (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
                  std::numeric_limits<double>::max(),
                  std::numeric_limits<double>::max())
                      .finished();
  } else {
    estStdDevs = estStdDevs * (1 + (avgDist.value() * avgDist.value() / 30));
  }
  return estStdDevs;
}

void Vision::SimPeriodic(frc::Pose2d robotSimPose) 
{
  visionSim->Update(robotSimPose);
}

void Vision::ResetSimPose(frc::Pose2d pose) 
{
  if (frc::RobotBase::IsSimulation()) {
    visionSim->ResetRobotPose(pose);
  }
}

frc::Field2d& Vision::GetSimDebugField() 
{ 
  return visionSim->GetDebugField(); 
}