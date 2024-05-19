#pragma once

#include "../Constants.hpp"

#include "AHRS.h" 

#include <frc2/command/SubsystemBase.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>

#include <frc/controller/PIDController.h>

// I know I have too many of these, but at this point im done
#include <units/velocity.h>
#include <units/base.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/energy.h>
#include <units/force.h>

#include <rev/CANSparkMax.h>

#include <ctre/phoenix/sensors/CANCoder.h>
#include <ctre/phoenix6/TalonFX.hpp>

using namespace ctre::phoenix6::hardware;
using namespace rev;
using namespace frc;

class Swerve : public frc2::SubsystemBase

{
    public:
        /// @brief Indicates the field centricity of the swerve drive.
        bool field_centered = true;

        /// @brief This is the current pose of the robot, as determined by odometry.Update
        Pose2d CurrentPose;

        /// @brief Current states for swerve telemetry.
        SwerveModuleState CurrentSwerveStates[4];

        // Used for FRC lib

        // Creating my odometry object from the kinematics object. Here,
        // our starting pose is 5 meters along the long end of the field and in the
        // center of the field along the short end, facing forward.

        units::radian_t initGyro{0};
        Rotation2d wpiinitGyro = Rotation2d(initGyro);
        // btw SwerveModulePosition is an frc class aka this is intended
        SwerveModulePosition defaultSwervePos{0_m, wpiinitGyro};

        SwerveDriveOdometry<4> m_odometry{
            m_driveKinematics,
            wpiinitGyro,
            {
                defaultSwervePos,
                defaultSwervePos,
                defaultSwervePos,
                defaultSwervePos,
            },
            Pose2d(),
        };

        /// @brief Constructor for the Swerve class.
        /// @param length - The length of the robot.
        /// @param width - The width of the robot.
        Swerve(float length, float width);

        /// @brief The method to drive the robot.
        /// @param y - The operator y stick value.
        /// @param x - The operator x stick value.
        /// @param x2 - The operator second x stick value.
        /// @param gyro - The robot gyro heading.
        void Drive(float y, float x, float x2);  // gyro is ignored when field_centered is false

        /// @brief Needed by odometry.
        /// @param desiredStates - input by the trajectory.
        void setModuleStates(std::array<SwerveModuleState, 4> desiredStates);

        /// @brief Method to set the wheens to the aboslute position.
        void Snap_Wheels_To_Absolute_Position();

        /// @brief Method to "x" the wheels, prevents the robot from moving. 
        void Toggle_X_Wheels();

        /// @brief ahhhhhhhhhhhhhhhhhhhhhhhhhhhhhh why didnt i just make this public??
        bool Get_X_Wheels();

        /// @brief Toggle SONIC MODE
	    void Toggle_Fast_Wheels();

        /// @brief litterally could've just made this a public variable
        bool Get_Fast_Wheels();

        /// @brief Create an attitude and heading reference system (AHRS).
        AHRS navx{frc::SerialPort::SerialPort::Port::kMXP};

        /// @brief uses gyro + motors to update CurrentPose
        /// @param gyroInput - just the current gyro output
        void updateOdometry(float gyroInput);

        /// @brief sets it to default init
        void resetOdometry(Pose2d pose);


        // this could've been in private
        // convert inches to meters for special princess WPIlib Swerve Kinematics
        units::meter_t LENGTH = 0.381000762_m;
        units::meter_t WIDTH  = 0.381000762_m;
        // Locations for the swerve drive modules relative to the robot center.
        frc::Translation2d m_frontLeftLocation{LENGTH, WIDTH};
        frc::Translation2d m_frontRightLocation{LENGTH, -WIDTH};
        frc::Translation2d m_backLeftLocation{-WIDTH, LENGTH};
        frc::Translation2d m_backRightLocation{-WIDTH, -LENGTH};

        // Creating my kinematics object using the module locations.
        frc::SwerveDriveKinematics<4> m_driveKinematics{
            m_frontLeftLocation, m_frontRightLocation, 
            m_backLeftLocation,m_backRightLocation
        };

    private:
        // @brief Method to create dead zones for the controller joysticks.
        /// @param x - Pointer to the x stick value to return the value used.
        /// @param y - Pointer to the y stick value to return the value used.
        /// @param x2 - Pointer to the second x stick value to return the value used.
        void deadzone_correction(float *x, float *y, float *x2);

        /// @brief  Converts funny silly Position Status Signal Object to "normal people units" aka schizo wpilib units lib
        /// @param rotations - Result from TalonFX GetPosition.GetValue()
        /// @return usable units for odometry
        units::meter_t rotationsToMeters(units::turn_t rotations);

        PIDController turningPidController{ANGLES_SWERVE_P, ANGLES_SWERVE_I, ANGLES_SWERVE_D};

        bool x_wheels = false;
	    bool fast_wheels = true;

        ctre::phoenix6::hardware::TalonFX FR_MOTOR_M{FR_M_CAN_ID, CANBUS_NAME};
        ctre::phoenix6::hardware::TalonFX FL_MOTOR_M{FL_M_CAN_ID, CANBUS_NAME};
        ctre::phoenix6::hardware::TalonFX RL_MOTOR_M{RL_M_CAN_ID, CANBUS_NAME};
        ctre::phoenix6::hardware::TalonFX RR_MOTOR_M{RR_M_CAN_ID, CANBUS_NAME};

        ctre::phoenix6::hardware::TalonFX *DRIVE_MOTORS[4] = {
            &FR_MOTOR_M,
            &FL_MOTOR_M,
            &RL_MOTOR_M,
            &RR_MOTOR_M,
        };

        CANSparkMax FR_MOTOR_A{FR_A_CAN_ID, CANSparkLowLevel::MotorType::kBrushless};
        CANSparkMax FL_MOTOR_A{FL_A_CAN_ID, CANSparkLowLevel::MotorType::kBrushless};
        CANSparkMax RL_MOTOR_A{RL_A_CAN_ID, CANSparkLowLevel::MotorType::kBrushless};
        CANSparkMax RR_MOTOR_A{RR_A_CAN_ID, CANSparkLowLevel::MotorType::kBrushless};

        CANSparkMax *ANGLE_MOTORS[4] = 
        {
            &FR_MOTOR_A,
            &FL_MOTOR_A,
            &RL_MOTOR_A,
            &RR_MOTOR_A,
        };

        SparkRelativeEncoder  *ANGLE_ENCODERS[4];
        SparkMaxPIDController *PID_CONTROLLERS[4];

        ctre::phoenix::sensors::CANCoder ANGLE_ABS_ENCODERS[4]
        {
            FR_E_A_CAN_ID,
            FL_E_A_CAN_ID,
            RL_E_A_CAN_ID,
            RR_E_A_CAN_ID,
        };

        // ctre::phoenix::sensors::CANCoder DRIVER_ABS_ENCODERS[4]
        // {
        //     FR_E_M_CAN_ID,
        //     FL_E_M_CAN_ID,
        //     RL_E_M_CAN_ID,
        //     RR_E_M_CAN_ID,
        // };
};
