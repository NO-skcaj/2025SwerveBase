
#include "../../include/io/Telemetry.hpp"

using namespace nt;

 /// @brief Constructor for the Telemetry class.
 /// @param swerve - Pointer to the swerve drive class.
Telemetry::Telemetry(Swerve *swerve)
{
    // Remember the swerve pointer
    this->m_swerve = swerve;

    m_chooser.SetDefaultOption(kAuto_Do_Nothing,  kAuto_Do_Nothing);
    m_chooser.AddOption(kAuto_Move,               kAuto_Move);
    m_chooser.AddOption(kAuto_Do_Nothing,         kAuto_Do_Nothing);

    SmartDashboard::PutData("Auto Modes", &m_chooser);
    
    // DEFENK
    SmartDashboard::PutBoolean("XWHEELS? ", this->m_swerve->Get_Fast_Wheels());

    // SONIC???
    SmartDashboard::PutBoolean("Sonic Mode??? ", this->m_swerve->Get_Fast_Wheels());
    
    // Direction the robot is facing
    SmartDashboard::PutNumber("Driver Angle: ", this->m_swerve->navx.GetYaw());

    SwerveSendable swerveData(this->m_swerve);

    SmartDashboard::PutData("Swerve Drive", (wpi::Sendable*)&swerveData);
    
    // Funny camera Pose
    frc::SmartDashboard::PutData("Field", &m_field);
}

// @brief Method called periodically
void Telemetry::Robot_Periodic()
{
    // DEFENK
    SmartDashboard::PutBoolean("XWHEELS? ", this->m_swerve->Get_Fast_Wheels());

    // SONIC???
    SmartDashboard::PutBoolean("Sonic Mode??? ", this->m_swerve->Get_Fast_Wheels());
    
    // Direction the robot is facing
    SmartDashboard::PutNumber("Driver Angle: ", this->m_swerve->navx.GetYaw());

    SwerveModuleState states[4] =  {this->m_swerve->CurrentSwerveStates[0], this->m_swerve->CurrentSwerveStates[1], this->m_swerve->CurrentSwerveStates[2], this->m_swerve->CurrentSwerveStates[3]};
    publisher.Set(states);

    // update position
    // this->posePublisher.Set(this->m_swerve->m_odometry.GetPose());

    SwerveSendable swerveData(this->m_swerve);

    SmartDashboard::PutData("Swerve Drive", (wpi::Sendable*)&swerveData);

    // Camera stuff; outputting estimated pose
    auto RobotPose = vision.GetEstimated().first;

    // Do this in either robot periodic or subsystem periodic
    m_field.SetRobotPose(frc::Pose2d(RobotPose.X, RobotPose.Y, RobotPose.Rotation()));
}