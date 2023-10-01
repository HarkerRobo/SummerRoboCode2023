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
    //motors on the swerve modules
    private TalonFX translation;
    private TalonFX rotation; 

    private CANCoder canCoder;

    //swerve module id
    private int ID;

    private SimpleMotorFeedforward feedforward;

    public SwerveModule(int id) {
        ID = id;

        //configures translation and rotation motors
      //  translation= new TalonFX(id);
        //translation.setInverted(RobotMap.SwerveModule.TRANSLATION_INVERTS[id]);
        translation = new HSFalconBuilder()
            .invert(RobotMap.SwerveModule.TRANSLATION_INVERTS[id])
            .supplyLimit(RobotMap.SwerveModule.TRANS_PEAK, RobotMap.SwerveModule.TRANS_CONTINUOUS, RobotMap.SwerveModule.TRANS_PEAK_DUR)
            .build(RobotMap.SwerveModule.TRANSLATION_IDS[id], RobotMap.CAN_CHAIN);

        rotation = new HSFalconBuilder()
            .invert(RobotMap.SwerveModule.ROTATION_INVERTS[id])
            .supplyLimit(RobotMap.SwerveModule.ROT_PEAK, RobotMap.SwerveModule.ROT_CONTINUOUS, RobotMap.SwerveModule.ROT_PEAK_DUR)
            .build(RobotMap.SwerveModule.ROTATION_IDS[id],RobotMap.CAN_CHAIN);
        
        canCoder = new CANCoder(RobotMap.SwerveModule.CAN_CODER_ID[id]);

        feedforward = new SimpleMotorFeedforward(RobotMap.SwerveModule.TRANSLATION_KS, RobotMap.SwerveModule.TRANSLATION_KV, RobotMap.SwerveModule.TRANSLATION_KA);
        
        init();
    }   
    /**
     * Sets cancoders to default
     */
    private void init() {
        translation.clearStickyFaults();

        rotation.clearStickyFaults();

        translation.configFactoryDefault();
        rotation.configFactoryDefault();

        rotation.config_kP(Constants.SLOT_INDEX, RobotMap.SwerveModule.ROTATION_KP);
        translation.config_kP(Constants.SLOT_INDEX, RobotMap.SwerveModule.TRANSLATION_KP);
        translation.config_kI(Constants.SLOT_INDEX, RobotMap.SwerveModule.TRANSLATION_KI);
        translation.config_kD(Constants.SLOT_INDEX, RobotMap.SwerveModule.TRANSLATION_KD);
        translation.enableVoltageCompensation(true); //disables voltage compensation 
        translation.configVelocityMeasurementWindow(RobotMap.SwerveModule.VELOCITY_WINDOW); //number of samples measured 
        translation.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
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
    public void setAngleAndDrive(SwerveModuleState state) {
        state = optimize(state);
        translation.set(ControlMode.Velocity, state.speedMetersPerSecond / RobotMap.SwerveModule.VELOCITY_CONVERSION, DemandType.ArbitraryFeedForward, feedforward.calculate(state.speedMetersPerSecond)/Constants.MAX_VOLTAGE);
        rotation.set(ControlMode.Position, state.angle.getDegrees() / RobotMap.SwerveModule.ROTATION_CONVERSION);
    }
    /*
     * adjusts the angle of a swerve module state 
     */
    public SwerveModuleState optimize (SwerveModuleState desiredState){
       double currentAngle = Math.toRadians(getAngle());
       double targetAngle = Math.IEEEremainder(desiredState.angle.getRadians(), Math.PI * 2);
       double remainder = currentAngle % (Math.PI * 2);
        var adjusted = targetAngle + currentAngle - remainder;

        var speed = desiredState.speedMetersPerSecond;
        SmartDashboard.putNumber(swerveIDToName(ID) + "Desired Translation Speed", speed);
        
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
    private void setAbsolutePosition() {
        double position = canCoder.getAbsolutePosition() - RobotMap.SwerveModule.CAN_CODER_OFFSETS[ID];
        rotation.setSelectedSensorPosition(position / RobotMap.SwerveModule.ROTATION_CONVERSION);
        zeroTranslation();
    }

    public void zeroTranslation() {
        translation.setSelectedSensorPosition(0);
    }
    /*
     * returns the angle of the rotation motor 
     */
    public double getAngle() {
        return rotation.getSelectedSensorPosition() * RobotMap.SwerveModule.ROTATION_CONVERSION;
    }
    /*
     * return speed of translation motor 
     */
    public double getSpeed() {
        return translation.getSelectedSensorVelocity() * RobotMap.SwerveModule.VELOCITY_CONVERSION;
    }
    /*
     * returns position of translation motor
     */
    public double getWheelPosition() {
        return translation.getSelectedSensorPosition() * RobotMap.SwerveModule.POSITION_CONVERSION;
    }

    //returns position and angle
    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getWheelPosition(), Rotation2d.fromDegrees(getAngle()));
    }
    //returns speed and angle
    public SwerveModulePosition getSwerveModuleState() {
        return new SwerveModulePosition(getSpeed(), Rotation2d.fromDegrees(getAngle()));
    }
    //name of module on smart dashbaord 
    public static String swerveIDToName(int swerveID) {
        String output = "";
        if (swerveID < 2) output += "Front ";
        else output += "Back ";
        if (swerveID % 2 == 0) output += "Left";
        else output += "Right";
        return output;
    }
}
