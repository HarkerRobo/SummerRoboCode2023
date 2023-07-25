package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import harkerrobolib.util.HSFalconBuilder;
import harkerrobolib.wrappers.HSFalcon;

public class SwerveModule {
    //motors on the swerve modules
    private HSFalcon translation;
    private HSFalcon rotation; 

    private CANCoder canCoder;

    //swerve module id
    private int ID;

    public SwerveModule(int id){
        ID = id;

        //configures translation and rotation motors
        translation = new HSFalconBuilder().invert(RobotMap.SwerveModule.TRANSLATION_INVERTS[id]).supplyLimit
            (RobotMap.SwerveModule.TRANS_PEAK, RobotMap.SwerveModule.TRANS_CONTINUOUS, RobotMap.SwerveModule.TRANS_PEAK_DUR).build
            (RobotMap.SwerveModule.TRANSLATION_IDS[id], RobotMap.CAN_CHAIN);

        rotation = new HSFalconBuilder().invert(RobotMap.SwerveModule.ROTATION_INVERTS[id]).supplyLimit
            (RobotMap.SwerveModule.ROT_PEAK, RobotMap.SwerveModule.ROT_CONTINUOUS, RobotMap.SwerveModule.ROT_PEAK_DUR).build
            (RobotMap.SwerveModule.ROTATION_IDS[id],RobotMap.CAN_CHAIN);

        init();
    }   
    /**
     * Sets cancoders to default
     */
    private void init(){
        rotation.config_kP(0, RobotMap.SwerveModule.kP);
        translation.enableVoltageCompensation(false); //disables voltage compensation 
        translation.configVelocityMeasurementWindow(32); //number of samples measured 
        canCoder.configFactoryDefault();
        canCoder.clearStickyFaults();
        canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        canCoder.setPositionToAbsolute();
        canCoder.configSensorDirection(false);
        setAbsolutePosition();
        
    }
    /**
     * Sets translation and rotation motors to move to new state
     * @param state new state 
     */
    public void setAngleAndDrive(SwerveModuleState state){
        state = optimize(state);
        //translation.setVoltage(transLoop.getVoltage(state.speedMetersPerSecond, getSpeed()));
        rotation.set(ControlMode.Position,state.angle.getDegrees()/RobotMap.SwerveModule.ROTATION_CONVERSION);
    }
    /*
     * adjusts the angle of a swerve module state 
     */
    public SwerveModuleState optimize (SwerveModuleState desiredState){
       double currentAngle = Math.toRadians(getAngle());
       double targetAngle = Math.IEEEremainder(desiredState.angle.getRadians(), Math.PI*2);
       double remainder = currentAngle%(Math.PI/2);
        var adjusted = targetAngle + currentAngle - remainder;

        var speed = desiredState.speedMetersPerSecond;
        SmartDashboard.putNumber(moduleName(ID) + "Desired Speed", speed);
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
    /*
     * resets the swerve module
     */
    private void setAbsolutePosition(){
        double position = canCoder.getAbsolutePosition()-RobotMap.SwerveModule.CAN_CODER_OFFSETS[ID];
        rotation.setSelectedSensorPosition(position/RobotMap.SwerveModule.ROTATION_CONVERSION);
        zeroTranslation();
    }
    public void zeroTranslation(){
        translation.setSelectedSensorPosition(0);
    }
    /*
     * returns the angle of the rotation motor 
     */
    public double getAngle(){
        return rotation.getSelectedSensorPosition()*RobotMap.SwerveModule.ROTATION_CONVERSION;
    }
    /*
     * return speed of translation motor 
     */
    public double getSpeed(){
        return translation.getSelectedSensorVelocity()*RobotMap.SwerveModule.VELOCITY_CONVERSION;
    }
    /*
     * returns position of translation motor
     */
    public double getPosition(){
        return translation.getSelectedSensorPosition()*RobotMap.SwerveModule.POSITION_CONVERSION;
    }
    //returns position and angle
    public SwerveModulePosition getCurrentPosition(){
        return new SwerveModulePosition(getPosition(),Rotation2d.fromDegrees(getAngle()));
    }
    //returns speed and angle
    public SwerveModulePosition getCurrentState(){
        return new SwerveModulePosition(getSpeed(),Rotation2d.fromDegrees(getAngle()));
    }
    //name of module on smart dashbaord 
    public static String moduleName(int id){
        String output = "motor id" + id;
        return output; 
    }
}
