#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>

#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ProfiledPIDCommand.h>
#include <frc2/command/InstantCommand.h>

#include <frc/controller/PIDController.h>

// I know I have too many of these, but at this point im done
#include <units/velocity.h>
#include <units/base.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/energy.h>
#include <units/force.h>

#include "../../include/commands/AutoCommands.hpp"
#include "../../include/subsystems/Swerve.hpp"

using namespace frc;
using namespace frc2;

/// @brief Constructor for the AutoCommands class.
/// @param swerve - Pointer to the robot swerve subsystem.
/// @param intake - Pointer to the robot intake subsystem.
AutoCommands::AutoCommands(Swerve *swerve, Intake *intake)
{
    // Remember the swerve and intake subsystem pointers
    m_swerve = swerve;
    m_intake = intake;
}


/// @brief This function does something *SPECIAL*. This basically creates the all of the commands for the auto. 
// Basically, it creates a trajectory then processes that into a Swerve Controller Command which then can be
// used to be put into a sequential command group. You can think of the Sequential Command Group as an array (more accurately a vector)
// of commands that get run. So you could bascially just put in any random command, function, or method that you want.
// This allows for a very large array of options that you can have for this. 
// As an example you could put in commands to raise, lower, and fire your launcher to score points in between commands to fire.
// SequentialCommandGroup AutoCommands::getAutonomousCommand() {

//     // 1. Create trajectory settings
//     TrajectoryConfig trajectoryConfig{
//         units::meters_per_second_t        {MAX_SPEED_MPS},
//         units::meters_per_second_squared_t{MAX_ACCELERATION_MPS},
//     }; //.setKinematics(this->m_swerve->m_driveKinematics);

//     // 2. Generate example trajectory
//     // in the future when we can actually have real autos we can use input
//     // from the driver station to determine what this would be.
//     Trajectory trajectory;
//     trajectory = TrajectoryGenerator::GenerateTrajectory(
//         Pose2d{0_m, 0_m, 0_deg},
//         { // midpoints
//             Translation2d(1_m, 0_m),
//             Translation2d(1_m, -1_m),
//         },
//         Pose2d(2_m, -1_m, Rotation2d(180_deg)), // end point
//         trajectoryConfig
//     );

//     // How does this work? What is a trapezoid? I have no idea, I failed geometry.
//     TrapezoidProfile<units::radians>::Constraints thetaConstraints{
//         90_rad_per_s,
//         45_rad_per_s / 1_s,
//     };
    
//     // 3. Define PID controllers for tracking trajectory
//     PIDController xController(SWERVE_P, SWERVE_I, SWERVE_D);
//     PIDController yController(SWERVE_P, SWERVE_I, SWERVE_D);
//     ProfiledPIDController<units::radians> thetaController{ANGLES_SWERVE_P, ANGLES_SWERVE_I, ANGLES_SWERVE_D, thetaConstraints};

//     thetaController.EnableContinuousInput(units::meter_t(-3.14159), units::meter_t(3.14159)); // again this is just pi idc enough to const

//     // 4. Construct command to follow trajectory
//     SwerveControllerCommand<4> swerveControllerCommand(
//         trajectory,
//         [this]() -> Pose2d {return this->m_swerve->m_odometry.GetPose();},
//         this->m_swerve->m_driveKinematics,
//         xController,
//         yController,
//         thetaController,
//         [this](wpi::array<SwerveModuleState, 4> desiredStates) -> void {this->m_swerve->setModuleStates(desiredStates);},
//         {this->m_swerve}.ToPtr()
//     ).ToPtr();

//     cmd::Sequence CommandListsss(
//         InstantCommand( [this, initialPose = trajectory.InitialPose()]() {m_swerve->resetOdometry(initialPose);},{}).ToPtr(),
//         swerveControllerCommand,
//         InstantCommand([this] { m_swerve->Drive(0, 0, 0, this->m_swerve->navx.GetYaw()); }, {}).ToPtr()
//     );

//     return CommandListsss;
// }
