#pragma once

#include <unistd.h>

#include <frc2/command/SequentialCommandGroup.h>

#include "../subsystems/Swerve.hpp"
#include "../subsystems/Intake.hpp"

/// @brief Class to support autonomous commands.
class AutoCommands
{
    public:
        /// @brief Constructor for the AutoCommands class.
        /// @param swerve - Pointer to the robot swerve subsystem.
        /// @param intake - Pointer to the robot intake subsystem.
        AutoCommands(Swerve *swerve, Intake *intake);

        // frc2::SequentialCommandGroup getAutonomousCommand();

    private:
        /// @brief Pointer to the robot swerve subsystem.
        Swerve *m_swerve;

        /// @brief Pointer to the robot intake subsystem.
        Intake *m_intake;
};
