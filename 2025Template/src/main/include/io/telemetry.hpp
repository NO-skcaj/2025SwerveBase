#pragma once

#include <networktables/StructTopic.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/shuffleboard/SimpleWidget.h>

#include <frc/geometry/Pose2d.h>

#include "../Constants.hpp"
#include "../subsystems/Swerve.hpp"

using namespace nt;

class Telemetry
{
    public:
        /// @brief Constructor for the DriverController class.
        /// @param swerve - Pointer to the swerve drive class.
        Telemetry(Swerve *swerve);

        /// @brief Method called periodically every dirver/operator control packet.
        void Robot_Periodic();

        /// @brief Starts vision and outputs it to shuffleboard
        static void VisionInit();

    private:
        
        /// @brief Pointer to the swerver drive class.
        Swerve *m_swerve;

        /// @brief initializes NetworkTables
        // NetworkTableInstance inst = NetworkTableInstance::GetDefault();
        /// @brief this is where you can publish data specifically for Advantage Scope
        // std::shared_ptr<NetworkTable> AdvantageScope = inst.GetTable("AdvantageScope");
        /// @brief this is where you can publish data
        // std::shared_ptr<NetworkTable> Shuffleboard = inst.GetTable("Shuffleboard");

        // Advantage Scope data

        // StructPublisher<Pose2d> posePublisher = AdvantageScope->GetStructTopic<Pose2d>(
        //     inst.GetStructTopic(std::string_view("Position"))
        // ).Publish();

        // StructArrayPublisher<SwerveModuleState> swervePublisher = AdvantageScope->GetStructArrayTopic<SwerveModuleState>(
        //     NetworkTableInstance::GetDefault().GetStructArrayTopic(std::string_view("Swerve"))
        // ).Publish();

        // Shuffleboard data

        // frc::ShuffleboardTab basicTab = frc::Shuffleboard::GetTab("Telemetry");

        // // simple number, outputs the current heading of the robot; where its facing
        // GenericEntry heading = frc::Shuffleboard::GetTab("Telemetry").Add("Heading", std::span<float>(this->m_swerve->navx.GetYaw())).GetEntry();

        // // This auto updates... I think; outputs camera
        // frc::Shuffleboard::GetTab("Telemetry").AddCamera(std::string_view("Cams"), std::span<std::string>("http://roboRIO-TEAM-frc.local:1181"));

        

};
