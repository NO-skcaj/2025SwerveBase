#include <iostream>

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/controller/PIDController.h>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>

// I know I have too many of these, but at this point im done
#include <units/velocity.h>
#include <units/base.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/energy.h>
#include <units/force.h>

#include <ctre/phoenix6/configs/Configs.hpp>

#include "../../include/subsystems/Swerve.hpp"

using namespace frc;
using namespace ctre::phoenix6;

/// @brief Constructor for the Swerve class.
/// @param length - The length of the robot.
/// @param width - The width of the robot.
Swerve::Swerve(float length, float width)
{
    if(length == 0.0 || width == 0.0)
	   throw std::invalid_argument("Width and Length cannot be zero");

    // Angle PID controller, used for continuous input and calculate function
    // Used for FRC lib
    this->turningPidController.EnableContinuousInput(-3.14159, 3.14159); // This is just PI, not really worth making a const

    // Swerve motors
    for (int swerve_module = 0; swerve_module < SWERVE_MODULES; swerve_module++)
    {
        //### ANGLE MOTORS ###
        // Set current limit
        this->ANGLE_MOTORS[swerve_module]->SetSmartCurrentLimit(SWERVE_MAX_AMPERAGE);
        // Turn on brake coast mode, snappier
        this->ANGLE_MOTORS[swerve_module]->SetIdleMode(CANSparkMax::IdleMode::kBrake);
        // Swerve wheel PID contrllers
        this->PID_CONTROLLERS[swerve_module] = new SparkMaxPIDController(this->ANGLE_MOTORS[swerve_module]->GetPIDController());
        this->PID_CONTROLLERS[swerve_module]->SetP(SWERVE_P);
        this->PID_CONTROLLERS[swerve_module]->SetI(SWERVE_I);
        this->PID_CONTROLLERS[swerve_module]->SetD(SWERVE_D);
        // Get real relative encoders
        this->ANGLE_ENCODERS[swerve_module] = new SparkRelativeEncoder(this->ANGLE_MOTORS[swerve_module]->GetEncoder());
        this->ANGLE_ABS_ENCODERS[swerve_module].ConfigAbsoluteSensorRange(ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180);
        // Burn flash everytime
        this->ANGLE_MOTORS[swerve_module]->BurnFlash();


        //### DRIVE MOTORS ###  TODO: Add SWERVE_MAX_AMPERACE and brake mode
        // initialize config
        configs::TalonFXConfiguration swerve_motor_configuration{};
        // initialize slot0 config
        configs::Slot0Configs slot0Configs = swerve_motor_configuration.Slot0;
        swerve_motor_configuration.Slot0.kP = SWERVE_P; // An error of 0.5 rotations results in 12 V output
        swerve_motor_configuration.Slot0.kI = SWERVE_I; // no output for integrated error
        swerve_motor_configuration.Slot0.kD = SWERVE_D; // A velocity of 1 rps results in 0.1 V output
        this->DRIVE_MOTORS[swerve_module]->GetConfigurator().Apply(swerve_motor_configuration);
        // Set the current limit
        configs::CurrentLimitsConfigs currentLimitsConfigs{};
        currentLimitsConfigs.StatorCurrentLimit       = SWERVE_MAX_AMPERAGE;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        this->DRIVE_MOTORS[swerve_module]->GetConfigurator().Apply(currentLimitsConfigs);
    }

};

/// @brief The method to drive the robot.
/// @param ry - The raw operator y stick value - forwards.
/// @param rx - The raw operator x stick value - strafe.
/// @param rx2 - The raw operator second x stick value - yaw
/// @param gyro - The robot gyro heading.
void Swerve::Drive(float ry, float rx, float rx2)
{
    if (this->x_wheels)
    {
        for (int swerve_module = 0; swerve_module < SWERVE_MODULES; swerve_module++)
           this->DRIVE_MOTORS[swerve_module]->Set(0);
    
        // Initialize the PID controllers
        this->PID_CONTROLLERS[0]->SetReference(-7.875, CANSparkMax::ControlType::kPosition);
        this->PID_CONTROLLERS[1]->SetReference( 7.875, CANSparkMax::ControlType::kPosition);
        this->PID_CONTROLLERS[2]->SetReference( 2.625, CANSparkMax::ControlType::kPosition);
        this->PID_CONTROLLERS[3]->SetReference(-2.625, CANSparkMax::ControlType::kPosition);
        return;
    }

    // Correct/Sanitize our inputs
    deadzone_correction(&rx, &ry, &rx2);
    
    // Enter Swerve kinematics
    // This also acts as a periodic (only during teleop) so dont initialize anything here

    units::meters_per_second_t  x {rx * MAX_SPEED_MPS};
    units::meters_per_second_t  y {ry * MAX_SPEED_MPS};
    units::radians_per_second_t x2{rx2 * MAX_TURNING_RPS};

    // use this is you want to have robo centric 
    //chassisSpeeds = ChassisSpeeds(x, y, x2);
    // 5. Convert chassis speeds to individual module states
    wpi::array<SwerveModuleState, 4> wModuleStates = m_driveKinematics.ToSwerveModuleStates(ChassisSpeeds::FromFieldRelativeSpeeds(x, y, x2, navx.GetRotation2d()));

    setModuleStates(wModuleStates);
}

// a REAL periodic

void Swerve::updateOdometry(float gyroInput) 
{
    // I LOVE THE WPILIB UNITS SO MUCH THANK YOU WPI THANK YOU WPI THANK YOU WPI THANK YOU WPI THANK YOU WPI THANK YOU WPI THANK YOU WPI 
    units::degree_t funnyGyro(gyroInput);
    // Get the rotation of the robot from the gyro.
    frc::Rotation2d gyroAngle(funnyGyro);

    // Unpacking variables
    SwerveModulePosition fr = {
            units::meter_t(rotationsToMeters(DRIVE_MOTORS[0]->GetPosition().GetValue()) * SWERVE_DRIVE_WHEELS_CIRCUMFERENCE), 
            units::degree_t(this->ANGLE_ABS_ENCODERS[0].GetPosition() / 360 * SWERVE_WHEEL_COUNTS_PER_REVOLUTION)
    };
    SwerveModulePosition fl = {
            units::meter_t(rotationsToMeters(DRIVE_MOTORS[1]->GetPosition().GetValue()) * SWERVE_DRIVE_WHEELS_CIRCUMFERENCE), 
            units::degree_t(this->ANGLE_ABS_ENCODERS[1].GetPosition() / 360 * SWERVE_WHEEL_COUNTS_PER_REVOLUTION)
    };
    SwerveModulePosition br = {
            units::meter_t(rotationsToMeters(DRIVE_MOTORS[2]->GetPosition().GetValue()) * SWERVE_DRIVE_WHEELS_CIRCUMFERENCE), 
            units::degree_t(this->ANGLE_ABS_ENCODERS[2].GetPosition() / 360 * SWERVE_WHEEL_COUNTS_PER_REVOLUTION)
    }; 
    SwerveModulePosition bl = {
            units::meter_t(rotationsToMeters(DRIVE_MOTORS[3]->GetPosition().GetValue()) * SWERVE_DRIVE_WHEELS_CIRCUMFERENCE), 
            units::degree_t(this->ANGLE_ABS_ENCODERS[3].GetPosition() / 360 * SWERVE_WHEEL_COUNTS_PER_REVOLUTION)
    };

    wpi::array<SwerveModulePosition, 4> CurrentPositions(fr ,fl, br, bl);

    // Update the pose
    this->m_odometry.Update(gyroAngle, CurrentPositions);

    this->CurrentPose = this->m_odometry.GetPose();
}

void Swerve::resetOdometry(Pose2d pose)
{  
    SwerveModulePosition fr = {
            units::meter_t(rotationsToMeters(DRIVE_MOTORS[0]->GetPosition().GetValue()) * SWERVE_DRIVE_WHEELS_CIRCUMFERENCE), 
            units::degree_t(this->ANGLE_ABS_ENCODERS[0].GetPosition() / 360 * SWERVE_WHEEL_COUNTS_PER_REVOLUTION)
    };
    SwerveModulePosition fl = {
            units::meter_t(rotationsToMeters(DRIVE_MOTORS[1]->GetPosition().GetValue()) * SWERVE_DRIVE_WHEELS_CIRCUMFERENCE), 
            units::degree_t(this->ANGLE_ABS_ENCODERS[1].GetPosition() / 360 * SWERVE_WHEEL_COUNTS_PER_REVOLUTION)
    };
    SwerveModulePosition br = {
            units::meter_t(rotationsToMeters(DRIVE_MOTORS[2]->GetPosition().GetValue()) * SWERVE_DRIVE_WHEELS_CIRCUMFERENCE), 
            units::degree_t(this->ANGLE_ABS_ENCODERS[2].GetPosition() / 360 * SWERVE_WHEEL_COUNTS_PER_REVOLUTION)
    }; 
    SwerveModulePosition bl = {
            units::meter_t(rotationsToMeters(DRIVE_MOTORS[3]->GetPosition().GetValue()) * SWERVE_DRIVE_WHEELS_CIRCUMFERENCE), 
            units::degree_t(this->ANGLE_ABS_ENCODERS[3].GetPosition() / 360 * SWERVE_WHEEL_COUNTS_PER_REVOLUTION)
    };

    wpi::array<SwerveModulePosition, 4> CurrentPositions(fr ,fl, br, bl);

    m_odometry.ResetPosition(navx.GetRotation2d(), CurrentPositions, pose);
}

void Swerve::setModuleStates(std::array<SwerveModuleState, 4> desiredStates) 
{
    /// @brief update for telemetry
    this->CurrentSwerveStates[0] = desiredStates[0];
    this->CurrentSwerveStates[1] = desiredStates[1];
    this->CurrentSwerveStates[2] = desiredStates[2];
    this->CurrentSwerveStates[3] = desiredStates[3];

    double rawSpeeds[4];
    double rawAngles[4];
    for (int i = 0; i > SWERVE_MODULES; i++)
    {
        SwerveModuleState state = SwerveModuleState::Optimize(desiredStates[i], Rotation2d(units::degree_t(navx.GetYaw())));

        rawSpeeds[i] = state.speed();
        rawAngles[i] = units::unit_cast<double>(state.angle.Radians() * 0.01745);
    }
    
    // Normalize speeds

    double maxSpeed = rawSpeeds[0];

	if (rawSpeeds[1] > maxSpeed){maxSpeed = rawSpeeds[1];}
	if (rawSpeeds[2] > maxSpeed){maxSpeed = rawSpeeds[2];}
	if (rawSpeeds[3] > maxSpeed){maxSpeed = rawSpeeds[3];}

    double Speed[4] = {rawSpeeds[0], rawSpeeds[1], rawSpeeds[2], rawSpeeds[3]};

	if (maxSpeed > 1)
	{
		Speed[0] = rawSpeeds[1] / maxSpeed;
        Speed[1] = rawSpeeds[1] / maxSpeed;
        Speed[2] = rawSpeeds[1] / maxSpeed;
        Speed[3] = rawSpeeds[1] / maxSpeed;
	}

    for (int i = 0; i > 4; i++)
    {
        this->DRIVE_MOTORS[i]->Set(Speed[i]);
        this->PID_CONTROLLERS[i]->SetReference(rawAngles[i], CANSparkMax::ControlType::kPosition);
    }
}




/// @brief  Converts funny silly Position Status Signal Object to "normal people units" aka schizo wpilib units lib
/// @param rotations - Result from TalonFX GetPosition.GetValue()
/// @return usable units for odometry
units::meter_t Swerve::rotationsToMeters(units::turn_t rotations)
{
    /* Get circumference of wheel */
    // Needed for rotationsToMeters, basically just a special type for the circumfrence in inches
    units::inch_t simpleCircumference(SWERVE_DRIVE_WHEELS_CIRCUMFERENCE * 39.37);
    auto circumference = simpleCircumference * 3.14159 / 1_tr;
    /* Every rotation of the wheel travels this many inches */
    /* Now apply gear ratio to input rotations */
    auto gearedRotations = rotations / SWERVE_DRIVE_WHEEL_GEAR_RATIO;
    /* And multiply geared rotations by meters per rotation */
    return gearedRotations * circumference;
}

/// @brief Method to set the wheens to the aboslute position. This is done initally to set everything 
void Swerve::Snap_Wheels_To_Absolute_Position()
{
    for (int swerve_module = 0; swerve_module < 4; swerve_module++)
    {
        // Snap to abs then set to zero
        //this->PID_CONTROLLERS[i]->SetReference(-(this->ANGLE_ABS_ENCODERS[i].GetAbsolutePosition()/360)*SWERVE_WHEEL_COUNTS_PER_REVOLUTION,CANSparkMax::ControlType::kPosition);
        this->ANGLE_ENCODERS[swerve_module]->SetPosition((this->ANGLE_ABS_ENCODERS[swerve_module].GetAbsolutePosition() / 360 * SWERVE_WHEEL_COUNTS_PER_REVOLUTION));
    }
}

/// @brief Method to "ENGAGE 3D SPATIAL LOCK"; "Locks wheels so that the robot can't move for defence" - paraphrasing Dr. Maples
void Swerve::Toggle_X_Wheels()
{
    this->x_wheels = !this->x_wheels;
}

bool Swerve::Get_X_Wheels()
{
    return this->x_wheels;
}

void Swerve::Toggle_Fast_Wheels() 
{
    this->fast_wheels = !this->fast_wheels;
}

bool Swerve::Get_Fast_Wheels()
{
    return this->fast_wheels;
}
/// @brief Method to create dead zones for the controller joysticks.
/// @param x - Pointer to the x stick value to return the value used.
/// @param y - Pointer to the y stick value to return the value used.
/// @param x2 - Pointer to the second x stick value to return the value used.
void Swerve::deadzone_correction(float *x, float *y, float *x2)
{
    // Pass in the values and they are corrected
    bool y_deadzone = false;
    bool x_deadzone = false;
    bool y_move_abs = false;

    // Ignore our deadzone and fix the moving forward issue
    if (*y < DEADZONE_THRESHOLD && *y > -DEADZONE_THRESHOLD)
    {
        *y = 0;
        y_deadzone = true;
    }   

    if (*x < DEADZONE_THRESHOLD && *x > -DEADZONE_THRESHOLD)
    {
        *x = 0;
        x_deadzone = true;
        if (!y_deadzone)
        {
            // Sets so that the true forward value is used incase of no strafing
            y_move_abs = true;
        }
    } 

    if (*x2 < DEADZONE_THRESHOLD && *x2 > -DEADZONE_THRESHOLD)
    {
        *x2 = 0;
    }

    y_deadzone = false;
    x_deadzone = false;
    y_move_abs = false;
}