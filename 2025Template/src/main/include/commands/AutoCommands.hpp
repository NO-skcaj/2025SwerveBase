#pragma once

#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ProfiledPIDCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/Commands.h>

#include "../subsystems/Swerve.hpp"
#include "../subsystems/Intake.hpp"

using namespace frc;
using namespace frc2;

/// @brief Class to support autonomous commands.
class AutoCommands
{
    public:
        /// @brief Constructor for the AutoCommands class.
        /// @param swerve - Pointer to the robot swerve subsystem.
        /// @param intake - Pointer to the robot intake subsystem.
        AutoCommands();

        frc2::CommandPtr getAutonomousCommand(Swerve *swerve, Intake *intake);

    private:
};
