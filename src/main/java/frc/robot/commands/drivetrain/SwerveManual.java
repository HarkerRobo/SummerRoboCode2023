package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.AngledElevator;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.Constants;
import harkerrobolib.util.MathUtil;

public class SwerveManual /* extends __ */ {

    // 5 instance variables
    // 2 for x and y velocities, 2 for previous x and y velocities, 1 for rotational velocity

    /**
     * Constructor for SwerveManual
     * Sets the requirements to Drivetrain
     * Initializes all instance variables to 0
     */
    public SwerveManual() {
    }

    public void execute() {
        // sets previous x and y velocities to current x and y velocities

        // get x, y, and rotational velocities from joystick; remember to use the deadband

        /**
         * vx will be to the left joystick's y value
         * vy will be to the left joystick's x value
         * 
         * omega will be to the right joystick's x value
         */

        // adjusts omega to the pigeon's yaw value

        /**
         * Scaling velocities based on multipliers
         * scale values based on the current velocity and the max driving speed
         *         if the elevator is far extended, scale the velocities by the clamp multiplier
         * 
         * scale rotational values based on the current rotational velocity and the max rotational velocity
         */


        // limits acceleration based on the previous velocity and the current velocity

        
        // aligns to nearest target
        // if (OI.getInstance().getDriver().getRightBumperState()) {
        //     omega = Drivetrain.getInstance().alignToTarget(omega);
        // }

        /**
         * if the robot is not visibly moving, set the x and y velocities to 0
         * checks if the robot is still by calling a method in this class
         * 
         * if the rotational velocity is very small, set it to a very small value
         *     this is to prevent the robot from not rotating at all (set it to 0.0001)
         */

        /**
         * sets the angle and drive to the calculated x, y, and rotational velocities
         * remember to use the field relative speeds
         */
    }

    /**
     * Limits the drivetrain's acceleration
     * @param value     velocity to correct
     * @param prevValue previous velocity
     * @return          corrected velocity
     */
    private double limitAcceleration(double value, double prevValue) {
        /**
         * if the absolute value of the difference between the current velocity and the previous velocity
         *     divided by the robot loop is greater than the max acceleration
         *          check if the elevator is far extended and set the max acceleration accordingly
         * 
         * set the current velocity to the previous velocity plus the sign of the difference between the current velocity and the previous velocity
         *    times the max acceleration times the robot loop
         *      remember to set the acceleration based on the elevator's position
         * 
         * previous velocity + direction of movement (+/-) * acceleration * time (a=v/t)
         */

        return 0;
    }

    /**
     * Scales the velocities by their given multiplier
     * @param value         velocity to scale
     * @param scaleFactor   multiplier
     * @return              scaled velocity
     */
    private double scaleValues() {
        return 0;
    }

    /**
     * Checks if the robot is moving slow enough to be considered still
     * Checks whether the robot's velocity is less than the minimum output
     *      the robot's velocity is calculated by the square root of the sum of the squares of the x and y velocities
     * @return if the robot is moving slow enough for it to be considered still
     */
    private boolean isRobotStill() {
        return false;
    }

    /**
     * Sets the x, y, and rotational velocities to 0.
     */
    public void end(boolean interrupted) {
        return;
    }
}