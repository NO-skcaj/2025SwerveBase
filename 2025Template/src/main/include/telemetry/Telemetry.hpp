#pragma once

#include <networktables/StructTopic.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/Field2d.h>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/shuffleboard/SimpleWidget.h>

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>

#include <networktables/GenericEntry.h>
#include <networktables/StructTopic.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>

#include <frc/kinematics/SwerveModuleState.h>

#include "../Constants.hpp"
#include "../subsystems/Swerve.hpp"
#include "../../include/io/TelemetryTypes.hpp"
#include "../../include/io/Vision.hpp"

using namespace nt;

class Telemetry
{
    public:
        /// @brief Constructor for the DriverController class.
        /// @param swerve - Pointer to the swerve drive class.
        Telemetry(Swerve *swerve);

        /// @brief Method called periodically every dirver/operator control packet.
        void Robot_Periodic();

    private:
        
        /// @brief Pointer to the swerver drive class.
        Swerve *m_swerve;

        Vision vision;

        /// @brief needed for field visualization
        frc::Field2d m_field;

        frc::Pose3d RobotPose = frc::Pose3d();

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

        // most if not all of these arent needed, its meant to be used by AdvantageScope
        StructArrayPublisher<frc::SwerveModuleState> publisher  = NetworkTableInstance::GetDefault().GetStructArrayTopic<frc::SwerveModuleState>("/why/MyStates").Publish();
        StructPublisher<frc::SwerveModuleState>      publisher1 = NetworkTableInstance::GetDefault().GetStructTopic<frc::SwerveModuleState>("/why/MyStates/State1").Publish();
        StructPublisher<frc::SwerveModuleState>      publisher2 = NetworkTableInstance::GetDefault().GetStructTopic<frc::SwerveModuleState>("/why/MyStates/State2").Publish();
        StructPublisher<frc::SwerveModuleState>      publisher3 = NetworkTableInstance::GetDefault().GetStructTopic<frc::SwerveModuleState>("/why/MyStates/State3").Publish();
        StructPublisher<frc::SwerveModuleState>      publisher4 = NetworkTableInstance::GetDefault().GetStructTopic<frc::SwerveModuleState>("/why/MyStates/State4").Publish();

        // Shuffleboard data
        
        const std::string kAuto_Do_Nothing = "Do Nothing";
        const std::string kAuto_Move       = "Auto Move";
        const std::string kAuto_Score_Red  = "Place Amp - Red";
        const std::string kAuto_Score_Blue = "Place Amp - Blue";

        // Sender for choosing the autonomous command.        
        frc::SendableChooser<std::string> m_chooser;
        std::string                       m_AutoCommandselected;

};
