#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/estimation/VisionEstimation.h>
#include <photon/simulation/VisionSystemSim.h>
#include <photon/simulation/VisionTargetSim.h>
#include <photon/targeting/PhotonPipelineResult.h>

#include <units/time.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>


std::shared_ptr<frc::AprilTagFieldLayout> Get2023Layout();

struct VisionConfig 
{
  std::shared_ptr<photon::PhotonCamera>     camera;
  frc::Transform3d                          robotToCamera;
  units::radian_t                           fov;
  std::shared_ptr<frc::AprilTagFieldLayout> layout;
};

class Vision 
{
    public:
    Vision(VisionConfig *config);
    ~Vision();

    VisionConfig *GetConfig();

    void OnStart();
    void OnUpdate(units::second_t dt);

    photon::PhotonPipelineResult                 GetLatestResult();
    std::span<const photon::PhotonTrackedTarget> GetTargets();
    photon::PhotonTrackedTarget                  GetBestTarget();
    frc::Pose3d                                  GetPose();
    frc::Transform3d                             GetPath(photon::PhotonTrackedTarget target);

    protected:
    private:
    VisionConfig *_config;
    std::vector<
        std::pair<std::shared_ptr<photon::PhotonCamera>, frc::Transform3d>>
        cameras = {std::make_pair(_config->camera, _config->robotToCamera)};
    photon::PhotonPoseEstimator _estimator = photon::PhotonPoseEstimator{
        Get2023Layout(), photon::CLOSEST_TO_REFERENCE_POSE, cameras};
    std::shared_ptr<nt::NetworkTable> table =nt::NetworkTableInstance::GetDefault().GetTable("Vision");
};