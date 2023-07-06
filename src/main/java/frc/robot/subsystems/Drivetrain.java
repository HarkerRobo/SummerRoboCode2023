package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Drivetrain extends SubsystemBase 
{
    private static Drivetrain instance;

    // TODO swerve modules

    private Pigeon2 pigeon;

    private SwerveDriveKinematics kinematics; // converts chassis speeds (x, y, theta) to module states (speed, angle)

    // Profiled PID for theta (turning) control
    private static final double THETA_P = 0.118;
    private static final double THETA_I = 0.0;
    private static final double THETA_D = 0.0;
    private static ProfiledPIDController thetaController = new ProfiledPIDController(THETA_P, THETA_I, THETA_D, new Constraints(4, 3.5));
    public static final double MAX_ERROR_YAW = 0.5;

    // Estimates the robot's pose through encoder (state) and vision measurements;
    private SwerveDrivePoseEstimator poseEstimator;

    // Standard deviations of pose estimate (x, y, heading)
    private static Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.2); // increase to trust encoder (state) measurements less
    private static Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.05, 0.025, 0.05); // increase to trust vsion measurements less

    private Drivetrain()
    {
        // initialize swerve modules

        // initialize pigeon
        pigeon = new Pigeon2(RobotMap.Drivetrain.PIGEON_ID);
        initPigeon();

        // initialize locations of swerve modules relative to robot (fl, fr, bl, br)
        kinematics = new SwerveDriveKinematics(
            new Translation2d(RobotMap.ROBOT_LENGTH / 2, RobotMap.ROBOT_WIDTH / 2),
            new Translation2d(RobotMap.ROBOT_LENGTH / 2, -RobotMap.ROBOT_WIDTH / 2),
            new Translation2d(-RobotMap.ROBOT_LENGTH / 2, RobotMap.ROBOT_WIDTH / 2),
            new Translation2d(-RobotMap.ROBOT_LENGTH / 2, -RobotMap.ROBOT_WIDTH / 2));

        // sets how much error to allow on theta controller
        thetaController.setTolerance(MAX_ERROR_YAW);

        // initial pose (holds the x, y, heading)
        Pose2d initalPoseMeters = new Pose2d();

        // initialize pose estimator
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            getRotation(),
            null,
            initalPoseMeters,
            stateStdDevs,
            visionStdDevs);
    }

    /*
     * Initialize pigeon values
     */
    private void initPigeon()
    {
        pigeon.configFactoryDefault();
        pigeon.configMountPoseYaw(90); // pigeon mounted at 90 degrees on robot
        pigeon.configMountPosePitch(0);
        pigeon.configMountPoseRoll(0);
        pigeon.setYaw(0); // sets yaw so driver's forward (0 degrees) = robot's forward
        pigeon.configEnableCompass(false);
    }

    /*
     * Returns yaw of pigeon in degrees (heading of robot)
     */
    public double getHeading()
    {
        SmartDashboard.putNumber("pigeon heading", pigeon.getYaw());
        return pigeon.getYaw();
    }

    /**
     * @return pitch of pigeon in degrees
     */
    public double getPitch()
    {
        return pigeon.getPitch();
    }

    /**
     * @return roll of pigeon in degrees
     */
    public double getRoll()
    {
        return pigeon.getRoll();
    }

    /**
     * @return heading of pigeon as a Rotation2d
     */
    public Rotation2d getRotation()
    {
        return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * @return the states of the swerve modules in an array
     */
    private SwerveModulePosition[] getModulePositions()
    {
        return null; //TODO
    }

    /**
     * @return kinematics of swerve drive
     */
    public SwerveDriveKinematics getKinematics()
    {
        return kinematics;
    }

    /**
     * Singleton code
     * @return instance of Drivetrain
     */
    public static Drivetrain getInstance()
    {
        if (instance == null)
        {
            instance = new Drivetrain();
        }
        return instance;
    }

    /**
     * Converts chassis speeds to individual swerve module 
     * states and sets the angle and drive for them
     * @param chassis       chassis speeds to convert
     */
    public void setAngleAndDrive(ChassisSpeeds chassis)
    {
        // TODO
    }

    /**
     * Called every loop, feeds newest encoder readings to estimator
     */
    public void updatePose()
    {
        poseEstimator.update(getRotation(), getModulePositions());
    }

    @Override
    public void periodic()
    {
        updatePose();
    }

    @Override
    public void initSendable(SendableBuilder builder)
    {
        builder.setSmartDashboardType("Drivetrain");
        builder.setActuator(true);
        // builder.addDoubleProperty("Pitch Value", () -> getPitch(), null);
        // builder.addDoubleProperty("Roll Value", () -> getRoll(), null);
    }
}
