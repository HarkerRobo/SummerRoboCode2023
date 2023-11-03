package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import harkerrobolib.util.Constants;
import harkerrobolib.util.HSFalconBuilder;
import harkerrobolib.wrappers.HSFalcon;

public class AngledElevator extends SubsystemBase {
    private static AngledElevator instance;

    private HSFalcon master;
    private HSFalcon follower;
    private DigitalInput limitSwitch;

    private double desiredPosition;

    private AngledElevator() {
        master = new HSFalconBuilder().invert(RobotMap.AngledElevator.MASTER_INVERTED).build(RobotMap.AngledElevator.MASTER_ID, RobotMap.CAN_CHAIN);
        follower = new HSFalconBuilder().invert(RobotMap.AngledElevator.FOLLOWER_INVERTED).build(RobotMap.AngledElevator.FOLLOWER_ID, RobotMap.CAN_CHAIN);
        
        // initialize 
        limitSwitch = new DigitalInput(RobotMap.AngledElevator.LIMIT_SWITCH_ID);
        initElevator();
    }


    /**
     * Initialized motors for AngledElevator
     */
    private void initElevator() {
        addChild("Master Motor", master);
        addChild("Follower Motor", follower);
        addChild("Limit Switch", limitSwitch);

        // make follower motor follow master motor
        follower.follow(master);

        master.configFactoryDefault();

        master.setNeutralMode(NeutralMode.Brake);
        master.config_kP(Constants.SLOT_INDEX, RobotMap.AngledElevator.kP);

        master.configForwardSoftLimitEnable(true);
        master.configReverseSoftLimitEnable(true);
        master.configForwardSoftLimitThreshold(RobotMap.AngledElevator.FORWARD_LIMIT);
        master.configReverseSoftLimitThreshold(RobotMap.AngledElevator.REVERSE_LIMIT);
        
        master.overrideSoftLimitsEnable(true);

        master.configMotionCruiseVelocity(RobotMap.AngledElevator.CRUISE_VELOCITY);
        master.configMotionAcceleration(RobotMap.AngledElevator.CRUISE_ACCELERATION);

        master.configClosedloopRamp(RobotMap.AngledElevator.RAMP_TIME);
    }

    /**
     * moves the Elevator to some desired position
     * @param   desiredPosition
     * @return  set the master motor using MotionMagic 
     */

    public void moveToPosition(double desiredPosition) {
        // this.desiredPosition = desiredPosition2;
        master.set(ControlMode.MotionMagic, desiredPosition, DemandType.ArbitraryFeedForward, RobotMap.AngledElevator.kG);
        SmartDashboard.putNumber("Elevator Desired", desiredPosition);
    }

    /**
     * @param   position
     * @return  set the desired position to some position
     */

    public void setDesiredPosition(double position) {
        this.desiredPosition = position;
    }

    /**
     * @return  desired position
     */

    public double getDesiredPosition() {
        return this.desiredPosition;
    }

    /**
     * check if the elevator is extended
     * @param   desiredposition 
     * @return  whether the absolute value of the desired position minus the current position is less than the max error of the elevator
     */

    public boolean checkExtend(double desiredposition) {
        return (Math.abs(desiredposition - getPosition()) < RobotMap.AngledElevator.MAX_ERROR);
    }

    /**
     * @return  get the selected encoder position of the master motor
     */

    public double getPosition() {
        return master.getSelectedSensorPosition();
    }

    /**
     * if power is 0, disable the motor
     * @param   power
     * @return  power to the motors using PercentOutput (constant); extends / retracts motors
     */

    public void setElevatorPower(double power) {
        if (power == 0)
            master.neutralOutput();
        else
            master.set(ControlMode.PercentOutput, power);
    }


    /**
     * @return  resetted encoders (set postion of the encoders to zero)
     */

    public void resetEncoders() {
        master.setSelectedSensorPosition(0);
        follower.setSelectedSensorPosition(0);
    }

    /**
     * @return  whether elevator is at ground level (at the bottom)
     */

    public boolean extensionStop() {
        return !limitSwitch.get();
    }

    /**
     * @return  if the elevator is extended to the max
     */
    public boolean isFarExtended() {
        return getPosition() > RobotMap.AngledElevator.POSITIONS[1];
    }

    /**
     * Singleton code
     * @return  isntance of AngledElevator
     */

    public static AngledElevator getInstance() {
        if (instance == null)
            instance = new AngledElevator();
        
        return instance;
    }

    /**
     * Smart Dashboard Function
     */

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Elevator");
        builder.setActuator(true);
        builder.setSafeState(() -> setElevatorPower(0));
        builder.addDoubleProperty("Current Elevator Position", this::getPosition, this::moveToPosition);
        builder.addBooleanProperty("Limit Switch", ()->limitSwitch.get(), null);
    }
}
