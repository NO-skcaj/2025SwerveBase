#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>

#include <frc/controller/PIDController.h>

#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ProfiledPIDCommand.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include <frc2/command/Requirements.h>

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
AutoCommands::AutoCommands(){}

/// @brief This function does something *SPECIAL*. This basically creates the all of the commands for the auto. 
// Basically, it creates a trajectory then processes that into a Swerve Controller Command which then can be
// used to be put into a sequential command group. You can think of the Sequential Command Group as an array (more accurately a vector)
// of commands that get run. So you could bascially just put in any random command, function, or method that you want.
// This allows for a very large array of options that you can have for this. 
// As an example you could put in commands to raise, lower, and fire your launcher to score points in between commands to fire.
/// @param swerve - swerve to use for all this
/// @param intake - not used but i might be after a sec of dev time
frc2::CommandPtr AutoCommands::getAutonomousCommand(Swerve *swerve, Intake *intake) 
{

  // Set up config for trajectory
  frc::TrajectoryConfig config(
    units::meters_per_second_t        {MAX_SPEED_MPS},
    units::meters_per_second_squared_t{MAX_ACCELERATION_MPS}
  );
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(swerve->m_driveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d{0_m, 0_m, 0_deg},
    {frc::Translation2d{1_m, 1_m}, frc::Translation2d{2_m, -1_m}},
    frc::Pose2d{3_m, 0_m, 0_deg},
    config
  );

  TrapezoidProfile<units::radians>::Constraints thetaConstraints{
    units::radians_per_second_t(90),
    units::radians_per_second_squared_t(45),
  };

  frc::ProfiledPIDController<units::radians> thetaController{
    ANGLES_SWERVE_P, ANGLES_SWERVE_I, ANGLES_SWERVE_D, 
    thetaConstraints
  };

  thetaController.EnableContinuousInput(
    units::radian_t{-std::numbers::pi},
    units::radian_t{std::numbers::pi}
  );

  frc2::CommandPtr swerveControllerCommand = frc2::SwerveControllerCommand<4>(
    exampleTrajectory, 
    [swerve]() { return swerve->Ulitmate_Pose_Estimation.GetEstimatedPosition(); },
    swerve->m_driveKinematics,
    frc::PIDController{SWERVE_P, SWERVE_I, SWERVE_D},
    frc::PIDController{SWERVE_P, SWERVE_I, SWERVE_D},
    thetaController,
    [swerve](auto moduleStates) {swerve->setModuleStates(moduleStates); }, // std::array

    {swerve}
  ).ToPtr();

  // Reset odometry to the initial pose of the trajectory, run path following
  // command, then stop at the end.
  auto seqeeenz = frc2::cmd::Sequence (
    frc2::InstantCommand(
      [swerve, initialPose = exampleTrajectory.InitialPose()]() {
      swerve->resetOdometry(initialPose);
      },{}
    ).ToPtr(),

    std::move(swerveControllerCommand),

    frc2::InstantCommand(
      [swerve] { swerve->Drive(0, 0, 0); }, {}
    ).ToPtr()
  );

  return seqeeenz;
}