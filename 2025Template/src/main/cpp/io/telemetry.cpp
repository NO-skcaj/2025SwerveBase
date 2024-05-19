#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/shuffleboard/SimpleWidget.h>

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>

#include <networktables/GenericEntry.h>
#include <networktables/StructTopic.h>

#include <cameraserver/CameraServer.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <thread>

#include "../../include/io/Telemetry.hpp"
#include "../../include/io/TelemetryTypes.hpp"

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
    
    // We need to run our vision in a separate thread for it still run. 
    // The actual function is nested inside the function, thread is being defined.
    std::thread VisionThread(VisionInit);
    VisionThread.detach();
}

void Telemetry::VisionInit() 
{
    // Get the USB camera from CameraServer
    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();

    // Set the resolution
    camera.SetResolution(640, 480);

    // Get a CvSink. This will capture Mats from the Camera
    cs::CvSink cvSink = frc::CameraServer::GetVideo();

    // Setup a CvSource. This will send images back to the Dashboard
    cs::CvSource outputStream = frc::CameraServer::PutVideo("Rectangle", 640, 480);

    // Mats are very memory expensive. Lets reuse this Mat.
    cv::Mat mat;

    while (true) 
    {
        // Tell the CvSink to grab a frame from the camera and put it in the
        // source mat. If there is an error notify the output.
        if (cvSink.GrabFrame(mat) == 0) 
        {
            // Send the output the error.
            outputStream.NotifyError(cvSink.GetError());

            // skip the rest of the current iteration
            continue;
        }

        // Put a rectangle on the image
        rectangle(mat, cv::Point(100, 100), cv::Point(400, 400), cv::Scalar(255, 255, 255), 5);

        // Give the output stream a new image to display
        outputStream.PutFrame(mat);
    }
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
}