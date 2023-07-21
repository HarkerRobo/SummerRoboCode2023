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
    private HSFalcon translation;
    private HSFalcon rotation; 

    private CANCoder canCoder;

    private int ID;

    private int kP = 0;

    private int kS = 0;
    private int kV = 0;
    private int kA = 0;

    public SwerveModule(int id){
        ID = id;

        translation = new HSFalconBuilder().invert(RobotMap.SwerveModule.TRANSLATION_INVERTS[id]).supplyLimit
            (RobotMap.SwerveModule.TRANS_PEAK, RobotMap.SwerveModule.TRANS_CONTINUOUS, RobotMap.SwerveModule.TRANS_PEAK_DUR).build
            (RobotMap.SwerveModule.TRANSLATION_IDS[id], RobotMap.CAN_CHAIN);

        rotation = new HSFalconBuilder().invert(RobotMap.SwerveModule.ROTATION_INVERTS[id]).supplyLimit
            (RobotMap.SwerveModule.ROT_PEAK, RobotMap.SwerveModule.ROT_CONTINUOUS, RobotMap.SwerveModule.ROT_PEAK_DUR).build
            (RobotMap.SwerveModule.ROTATION_IDS[id],RobotMap.CAN_CHAIN);

        init();
    }   

    private void init(){
        rotation.config_kP(0, kP);
        translation.enableVoltageCompensation(false);
        translation.configVelocityMeasurementWindow(32);
        canCoder.configFactoryDefault();
        canCoder.clearStickyFaults();
        canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        canCoder.setPositionToAbsolute();
        canCoder.configSensorDirection(false);
        setAbsolutePosition();
        
    }
    public void setAngleAndDrive(SwerveModuleState state){
        state = optimize(state);
        //translation.setVoltage(transLoop.getVoltage(state.speedMetersPerSecond, getSpeed()));
        rotation.set(ControlMode.Position,state.angle.getDegrees()/RobotMap.SwerveModule.ROTATION_CONVERSION);
    }
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
    private void setAbsolutePosition(){
        double position = canCoder.getAbsolutePosition()-RobotMap.SwerveModule.CAN_CODER_OFFSETS[ID];
        rotation.setSelectedSensorPosition(position/RobotMap.SwerveModule.ROTATION_CONVERSION);
        zeroTranslation();
    }
    public void zeroTranslation(){
        translation.setSelectedSensorPosition(0);
    }
    public double getAngle(){
        return rotation.getSelectedSensorPosition()*RobotMap.SwerveModule.ROTATION_CONVERSION;
    }
    public double getSpeed(){
        return translation.getSelectedSensorVelocity()*RobotMap.SwerveModule.VELOCITY_CONVERSION;
    }
    public double getPosition(){
        return translation.getSelectedSensorPosition()*RobotMap.SwerveModule.POSITION_CONVERSION;
    }
    public SwerveModulePosition getCurrentPosition(){
        return new SwerveModulePosition(getPosition(),Rotation2d.fromDegrees(getAngle()));
    }
    public SwerveModulePosition getCurrentState(){
        return new SwerveModulePosition(getSpeed(),Rotation2d.fromDegrees(getAngle()));
    }
    //not done based on ids
    public static String moduleName(int id){
        String output = "motor id" + id;
        return output; 
    }
}
