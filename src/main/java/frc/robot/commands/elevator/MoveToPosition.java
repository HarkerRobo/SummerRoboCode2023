package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AngledElevator;

public class MoveToPosition /* extends __ */ {

    // Instance Variables
    // The position to move the elevator to
    // Timer to keep track of how long the elevator has been moving; make sure its a final variable

    /**
     * Constructor for MoveToPosition
     * Associates MoveToPosition with AngledElevator subsystem
     * sets the instance variable position to the parameter position
     * @param position
     */
    public MoveToPosition() {
        /* CODE HERE */
    }

    /**
     * Resets the timer and starts it (stopwatch)
     * This is for later
     */
    public void initialize() {
        return;
    }

    /**
     * Moves the elevator to a set position
     * Make sure to use the method in AngledElevator
     */
    public void execute() {
        return;
    }

    /**
     * if the Elevator is at the right position or the timer is above 2.5 seconds, return true\
     * else return false
     */

    public boolean isFinished() {
        // Change
        return false;
    }

    /**
     * Keep the elevator at the set position and stop the power to the elevator
     */
    public void end(boolean interrupted) {
        return;
    }
}