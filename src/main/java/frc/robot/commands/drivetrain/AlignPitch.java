package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;

public class AlignPitch /* extends __ */ {

    /**
     * Creates a new ProfiledPIDController for the pitch controller
     * The pitch controller is used to align the robot's pitch
     * Sets the P, I, and D values to the constants in RobotMap
     * Sets the constraints to the max driving speed (for max velocity) and half of the max driving speed (for max acceleration)
     */

    /**
     * Creates a new AlignPitch command
     * Requires the Drivetrain subsystem
     */ 
    public AlignPitch() {
    }

    /**
     * Initializes the pitch controller by reseting its values and setting
     * its tolerance and setpoint
     *  both the tolerance and setpoint are constants in RobotMap
     */
    public void initialize() {
        return;
    }

    /**
     * Feeds the error into the pitch controller and moves forward by the
     * calculated velocity
     */
    public void execute() {
        // sets the error to the difference between the setpoint and the current roll value

        // sets the forward amount to the calculated velocity given the error

        // sets the omega to the adjusted pigeon value (see Drivetrain.java)

        // creates a new ChassisSpeeds object with the forward amount, 0, and 0

        // sets the angle and drive to the ChassisSpeeds object
    }

    /**
     * Stops command when the pitch value is within the error tolerance
     * of the setpoint
     * Make sure to override the method
     */
    public boolean isFinished() {
        // Change
        return false;
    }

    /**
     * Sets the x, y, and rotational velocities to 0.
     */
    public void end(boolean interrupted) {
        return;
    }
}
