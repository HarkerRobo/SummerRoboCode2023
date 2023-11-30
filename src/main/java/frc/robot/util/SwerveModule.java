package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import harkerrobolib.util.Constants;
import harkerrobolib.util.HSFalconBuilder;


public class SwerveModule {
    //motors on the swerve modules; two TalonFXs; translation and rotation

    // cancoder

    // swerve module id

    // feedforward using SimpleMotorFeedforward

    public SwerveModule(int id) {
        // sets the swerve module id to the id passed in

        // init translation and rotation motors using HSFalconBuilder; add supplyLimit using constants from RobotMap
        
        // initialize cancoder

        // initialize feedforward using SimpleMotorFeedforward and constants from RobotMap
        
        // call init method
    }

    /**
     * Sets cancoders to default
     */
    private void init() {
        // clears sticky faults for both translation and rotation motors

        // configs the translation and rotation motors to factory default

        // configs kP for rotation motor

        // configs kP, kI, and kD for translation motor

        // enable voltage compensation for translation motor

        // configs the velocity measurement window and period for translation motor (using constants from RobotMap
        // measurement window is 10ms)

        /**
         * Sets cancoder to default (configFactoryDefault)
         * Clears sticky faults
         * Configs sensor initialization strategy to boot to absolute position
         * Sets position to absolute
         * Configs sensor direction to false
         * Sets absolute position (call method)
         */
        
    }
    /**
     * Sets translation and rotation motors to move to new state
     * Try to write this without any references
     * If you are really stuck, refrence your neo drivetrain code
     * @param state new state 
     */
    public void setAngleAndDrive(SwerveModuleState state) {
        return;
    }

    /**
     * adjusts the angle of a swerve module state 
     */
    public SwerveModuleState optimize (SwerveModuleState desiredState){
       double currentAngle = Math.toRadians(getAngle());
       double targetAngle = Math.IEEEremainder(desiredState.angle.getRadians(), Math.PI * 2);
       double remainder = currentAngle % (Math.PI * 2);
        var adjusted = targetAngle + currentAngle - remainder;

        var speed = desiredState.speedMetersPerSecond;
        
        if(adjusted - currentAngle > Math.PI) {
            adjusted -= Math.PI * 2;
        }
        if (adjusted - currentAngle < -Math.PI) {
            adjusted += Math.PI * 2;
        }
        if (adjusted - currentAngle > Math.PI / 2) {
            adjusted -= Math.PI;
            speed *= -1;
        } else if (adjusted - currentAngle < -Math.PI / 2) {
            adjusted += Math.PI;
            speed *= -1;
        }
        return new SwerveModuleState(speed, Rotation2d.fromRadians(adjusted));
        
    }

    /**
     * resets the swerve module
     * sets the position of the canCoder based on the absolute position of the canCoder and the offset
     * set the sensor position of the rotation motor to the position of the canCoder divided by the conversion factor
     * zero the translation motor
     */
    private void setAbsolutePosition() {
        return;
    }

    // sets the sensor position of the translation motor to 0
    public void zeroTranslation() {
        return;
    }

    /**
     * returns the angle of the rotation motor (remember to use the conversion factor)
     */
    public double getAngle() {
        return 0;
    }

    /**
     * return speed of translation motor 
     * this is similar to getAngle()
     */
    public double getSpeed() {
        return 0;
    }

    /**
     * returns position of translation motor
     * this is similar to the methods above
     */
    public double getWheelPosition() {
        return 0;
    }
    
    // returns position and angle
    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition();
    }

    // returns speed and angle
    public SwerveModulePosition getSwerveModuleState() {
        return new SwerveModulePosition();
    }

    // name of module on smart dashbaord 
    public static String swerveIDToName(int swerveID) {
        String output = "";
        if (swerveID < 2) output += "Front ";
        else output += "Back ";
        if (swerveID % 2 == 0) output += "Left";
        else output += "Right";
        return output;
    }
}
