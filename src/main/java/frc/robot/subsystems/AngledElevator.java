package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import harkerrobolib.util.Constants;
import harkerrobolib.util.HSFalconBuilder;
import harkerrobolib.wrappers.HSFalcon;

public class AngledElevator /* extends __ */ {
    // instance variable for singleton

    // two motors, both HSFalcons; one master, one follower

    // limit switch, DigitalInput

    // instance variable for the desiredPosition of the elevator (double)

    private AngledElevator() {
        // initializes motors using HSFalconBuilder

        // initializes limit switch

        // calls initElevator to initialize the motors
    }

    /**
     * Initialized motors for AngledElevator
     */
    private void initElevator() {
        // make follower motor follow master motor

        // configs kP for the master motor

        // configs the softlimits for the motor and the thresholds (using constants from
        // RobotMap)

        // overrides the softlimits and sets them to true

        // configs the cruise velocity and acceleration for the motor

        // configs the ramp rate for the motor
    }

    /**
     * moves the Elevator to some desired position
     * set the master motor using MotionMagic and FeedForward
     * 
     * @param desiredPosition
     */

    public void moveToPosition(double desiredPosition) {
        return;
    }

    /**
     * set the desired position to some position
     * 
     * @param position
     */

    public void setDesiredPosition(double position) {
        return;
    }

    /**
     * Gets the desired position of the elevator and returns it
     * 
     * @return desired position
     */

    public double getDesiredPosition() {
        // Change
        return 0;
    }

    /**
     * check if the elevator is extended
     * 
     * @param desiredposition
     * @return whether the absolute value of the desired position minus the current
     *         position is less than the max error of the elevator
     */

    public boolean checkExtend(double desiredposition) {
        // Change
        return false;
    }

    /**
     * Gets the current position of the elevator and returns it (sensor position)
     * 
     * @return get the selected encoder position of the master motor
     */

    public double getPosition() {
        // Change
        return 0;
    }

    /**
     * if power is 0, disable the motor (neutralOutput)
     * power to the motors using PercentOutput (constant)
     * 
     * @param power
     */

    public void setElevatorPower(double power) {
        return;
    }

    /**
     * Resets the encoders for the motors
     * 
     * @postcondition reset encoders (set postion of the encoders to zero)
     */

    public void resetEncoders() {
        return;
    }

    /**
     * Check if the elevator is at the ground level using the limit switch
     * 
     * @return whether elevator is at ground level (at the bottom)
     */

    public boolean extensionStop() {
        // Change
        return false;
    }

    /**
     * Check if the elevator is at the max height (if the position is greater than
     * the highest position)
     * 
     * @return if the elevator is extended to the max
     */
    public boolean isFarExtended() {
        // Change
        return false;
    }

    /**
     * Singleton code
     * 
     * @return isntance of AngledElevator
     */
}
