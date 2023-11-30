package frc.robot;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.claw.ToggleClaw;
import frc.robot.commands.drivetrain.AlignPitch;
import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.commands.elevator.ZeroElevator;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.joysticks.XboxGamepad;
import harkerrobolib.util.Constants;

public class OI {
    // Instance variable for singleton

    // Driver and Operator Controllers (XboxGamepad); driver and operator

    /**
     * Constructor for OI
     * Creates Driver and Operator Controllers (use the respective IDs in Constants)
     * Initializes bindings
     */
    private OI() {
        /* CODE HERE */
    }

    /**
     * @return the driver instance
     */

    public XboxGamepad getDriver() {
        // Change Code
        return new XboxGamepad(0);
    }

    /**
     * @return the operator instance
     */

    public XboxGamepad getOperator() {
        // Change Code
        return new XboxGamepad(0);
    }

    private void initBindings() {
        // set LeftBumper Button to Align Pitch (driver)

        // set Start Button to Align Yaw (Reset Yaw) (driver)

        // set the right dpad button to toggle the claw (operator)

        // set the right bumper button to zero the elevator (operator)

        // set the left bumper button to move the elevator to the 4th position (operator) (soft zero)

        // set A button on OPERATOR controller to set Elevator to POSITION LOW
        // set B button on OPERATOR controller to set Elevator to POSITION MID
        // set Y button on OPERATOR controller to set Elevator to POSITION HIGH
        // set X button on OPERATOR controller to set Elevator to POSITION HUMAN PLAYER (HP)
    }

    /**
     * Singleton Code
     * 
     * @return instance of OI
     */
}