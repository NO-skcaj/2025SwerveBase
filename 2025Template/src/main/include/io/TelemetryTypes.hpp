#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/shuffleboard/SimpleWidget.h>

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveModuleState.h>

#include "../subsystems/Swerve.hpp"

/// @brief Swerve data
class SwerveSendable : public wpi::Sendable {
    public:
        SwerveSendable(Swerve *swerve) { this->m_swerve = swerve;}

        void InitSendable(wpi::SendableBuilder& builder) override {

            builder.SetSmartDashboardType("SwerveDrive");

            builder.AddDoubleProperty("Front Left Angle", [this]() -> double {return units::unit_cast<double>(m_swerve->CurrentSwerveStates[0].angle.Radians());}, nullptr);
            builder.AddDoubleProperty("Front Left Velocity", [this]() -> double {return units::unit_cast<double>(m_swerve->CurrentSwerveStates[0].speed);}, nullptr);
            builder.AddDoubleProperty("Front Right Angle", [this]() -> double {return units::unit_cast<double>(m_swerve->CurrentSwerveStates[1].angle.Radians());}, nullptr);
            builder.AddDoubleProperty("Front Right Velocity", [this]() -> double {return units::unit_cast<double>(m_swerve->CurrentSwerveStates[1].speed);}, nullptr);

            builder.AddDoubleProperty("Back Left Angle", [this]() -> double {return units::unit_cast<double>(m_swerve->CurrentSwerveStates[2].angle.Radians());}, nullptr);
            builder.AddDoubleProperty("Back Left Velocity", [this]() -> double {return units::unit_cast<double>(m_swerve->CurrentSwerveStates[2].speed);}, nullptr);

            builder.AddDoubleProperty("Back Right Angle", [this]() -> double {return units::unit_cast<double>(m_swerve->CurrentSwerveStates[3].angle.Radians());}, nullptr);
            builder.AddDoubleProperty("Back Right Velocity", [this]() -> double {return units::unit_cast<double>(m_swerve->CurrentSwerveStates[3].speed);}, nullptr);

            builder.AddDoubleProperty("Robot Angle", [this]() -> double {return units::unit_cast<double>(m_swerve->navx.GetRotation2d().Radians());}, nullptr);
        }
    private:
        Swerve *m_swerve;
};