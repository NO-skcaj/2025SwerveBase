#include <frc/smartdashboard/SmartDashboard.h>

#include "../../include/commands/OperatorController.hpp"

/// @brief Constructor for the OperatorController class.
/// @param intake - Pointer to the intake class to allow calling intake methods.
OperatorController::OperatorController(Intake *intake)
{
    // Remember the method parameters
    m_intake = intake;
}

/// @brief Method called periodically every dirver/operator control packet.
///
/// Operator:
///    JoyStick Y:          Intake rollers.
///
///    Button POV_0:        Intake move to Feed.
///    Button POV_180:      Intake move to Amp.
///
///    Button A:            Step intake negative.
///    Button B:            Intake to Climb Position.
///    Button Y:            Step intake positive.
///    Button X:            Flip intake to opposite position.
///
///    Button Bumper Right: Climb retract (Climb).
///    Button Bumper Left:  Climb extend.
///     
///     D-Pad
///    Button POV_90:       Enable cliber motors.
///    Button POV_270:      Disable cliber motors.
void OperatorController::Robot_Periodic()
{
    // Intake rollers
    double y  = -m_operator_joystick.GetY();
    if (y < INTAKE_ROLLER_MOTOR_JOYSTICK_DEADBAND && y > -INTAKE_ROLLER_MOTOR_JOYSTICK_DEADBAND)
        y = 0; 
    m_intake->Set_Roller_Motors(y * INTAKE_MAXIMUM_ROLLER_POWER);

    // Extend/retract the intake
    if (m_operator_joystick.GetPOV() == JOYSTICK_POV_0)
        m_intake->MoveToFeed();
    else if (m_operator_joystick.GetPOV() == JOYSTICK_POV_180)
        m_intake->MoveToAmp();

    // Check for intake angle lower offset (in case belt slips)
    if (m_operator_joystick.GetRawButtonPressed(JOYSTICK_BUTTON_A))
       m_intake->AddIntakeOffset(-INTAKE_POSITION_STEP_VALUE);
    
    // Check for intake angle lower offset (in case belt slips)
    if (m_operator_joystick.GetRawButtonPressed(JOYSTICK_BUTTON_B))
       m_intake->MoveToClimb();

    // Check for intake angle raise offset (in case belt slips)
    else if (m_operator_joystick.GetRawButtonPressed(JOYSTICK_BUTTON_Y))
        m_intake->AddIntakeOffset(INTAKE_POSITION_STEP_VALUE);

    // Flip the intake
    if (m_operator_joystick.GetRawButtonPressed(JOYSTICK_BUTTON_X))
        m_intake->Flip_Retraction();
}
