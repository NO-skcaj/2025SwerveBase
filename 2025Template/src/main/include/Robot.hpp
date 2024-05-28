// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <fmt/core.h>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <cameraserver/CameraServer.h>

#include <opencv2/imgproc/imgproc.hpp>

#include "../include/io/OperatorController.hpp"
#include "../include/io/DriverController.hpp"
#include "../include/commands/AutoCommands.hpp"

#include "../include/telemetry/Telemetry.hpp"



class Robot : public frc::TimedRobot
{
    public:
        void RobotInit()          override;
        void RobotPeriodic()      override;
        void AutonomousInit()     override;
        void AutonomousPeriodic() override;
        void TeleopInit()         override;
        void TeleopPeriodic()     override;
        void DisabledInit()       override;
        void DisabledPeriodic()   override;
        void TestInit()           override;
        void TestPeriodic()       override;
        void SimulationInit()     override;
        void SimulationPeriodic() override;
 
    private:
        // Robot subassemblies
        Swerve             SWERVE{CHASSIS_LENGTH, CHASSIS_WIDTH};
        Intake             INTAKE{};
        // Robot controllers (operator and controller)
        OperatorController O_CONTROLLER{&INTAKE};
        DriverController   D_CONTROLLER{&SWERVE};

        // Method to run autonomous commands
        AutoCommands       Autonomous{};

        // Telemetry things, adds data to Advantage Scope and Shuffleboard and take input for auto
        Telemetry          io{&SWERVE};
};
