
#include "../../include/telemetry/Telemetry.hpp"

using namespace nt;

 /// @brief Constructor for the Telemetry class.
 /// @param swerve - Pointer to the swerve drive class.
Telemetry::Telemetry(Swerve *swerve, bool team)
{
    // Remember the swerve pointer
    this->m_swerve = swerve;

    vision.VisionInit();

    m_chooser.SetDefaultOption(kAuto_Do_Nothing,  kAuto_Do_Nothing);
    m_chooser.AddOption(kAuto_Move,               kAuto_Move);
    m_chooser.AddOption(kAuto_Do_Nothing,         kAuto_Do_Nothing);

    SmartDashboard::PutData("Auto Modes", &m_chooser);

    m_chooser.SetDefaultOption(kRed, kRed);
    m_chooser.AddOption(kRed,        kRed);
    m_chooser.AddOption(kBlue,       kBlue);

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

    // I HAD ALL OF THIS FOR PHOTON VISION
    // AHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH
    // it should work now

    this->RobotPose = this->m_vision.currentPose;

    // Do this in either robot periodic or subsystem periodic
    this->m_field.SetRobotPose(this->RobotPose);
}