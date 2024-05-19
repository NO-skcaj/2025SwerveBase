#pragma once 

#include "../Constants.hpp"

#include <frc/Joystick.h>
#include <frc2/command/SubsystemBase.h>

#include "../subsystems/Intake.hpp"

class OperatorController : frc2::SubsystemBase
{
    public:
        /// @brief Constructor for the OperatorController class.
        /// @param intake - Pointer to the intake class to allow calling intake methods.
        OperatorController(Intake *intake);

        /// @brief Method called periodically every dirver/operator control packet.
        void Robot_Periodic(); 

    private:
        /// @brief Pointer to the intake class to allow calling intake methods.
        Intake *m_intake;

        /// @brief The operator controller (joystick).
        frc::Joystick m_operator_joystick{OPERATOR_CONTROLLER};
};
