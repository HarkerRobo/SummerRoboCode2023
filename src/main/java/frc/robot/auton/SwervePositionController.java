package frc.robot.auton;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

public class SwervePositionController extends CommandBase {

    /*
     * PID values for X, Y, and Rotation (THETA)
     */

    public static double X_kP = 3.0; // TUNE (based on RoboCode2023 values)
    public static double X_kI = 0.0;
    public static double X_kD = 0.0;

    public static double Y_kP = 2.5; // TUNE (based on RoboCode2023 values)
    public static double Y_kI = 0.0;
    public static double Y_kD = 0.0;

    public static double THETA_kP = 1.0; // TUNE (based on RoboCode2023 values)
    public static double THETA_kI = 0.0;
    public static double THETA_kD = 0.0;

    /*
     * PID Controllers for X, Y, and Rotation (THETA)
     */

    private static PIDController xController = new PIDController(X_kP, X_kI, X_kD);
    private static PIDController yController = new PIDController(Y_kP, Y_kI, Y_kD);
    private static PIDController thetaController = new PIDController(THETA_kP, THETA_kI, THETA_kD);

    private Trajectory trajectory;
    private Supplier<Rotation2d> referenceHeading;
    private Supplier<Rotation2d> startHeading;

    private final Timer timer = new Timer();

    public SwervePositionController(
            Trajectory trajectory, Supplier<Rotation2d> referenceHeading, Supplier<Rotation2d> startHeading) {
        this.trajectory = trajectory;
        this.referenceHeading = referenceHeading;
        this.startHeading = startHeading;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(Drivetrain.getInstance());
    }

    public SwervePositionController(Trajectory trajectory, Supplier<Rotation2d> referenceHeading) {
        this(trajectory, referenceHeading, null);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        xController.reset();
        yController.reset();
        thetaController.reset();
    }

    @Override
    public void execute() {
        xController.setP(X_kP);
        yController.setP(Y_kP);
        thetaController.setP(THETA_kP);

        Trajectory.State desiredState = Trajectories.apply(trajectory.sample(timer.get()));

        Rotation2d referenceAngle = Trajectories.apply(referenceHeading.get());

        Pose2d currentPose = Drivetrain.getInstance().getPoseEstimatorPose2d();
        Rotation2d currentRotation = Drivetrain.getInstance().getRotation();

        double clampAdd = 1 + Math.abs(referenceAngle.getRadians() - currentRotation.getRadians()) * (2 / Math.PI);

        /*
         * Feedforward values for X, Y, and Rotation (THETA)
         */

        double xFF = desiredState.velocityMetersPerSecond * desiredState.poseMeters.getRotation().getCos();
        double yFF = desiredState.velocityMetersPerSecond * desiredState.poseMeters.getRotation().getSin();

        double thetaFF = desiredState.velocityMetersPerSecond
                * MathUtil.clamp(thetaController.calculate(currentRotation.getRadians(), referenceAngle.getRadians()),
                        -clampAdd, clampAdd);

        /*
         * Feedback values for X, Y
         */
        double xFeedback = xController.calculate(currentPose.getX(), desiredState.poseMeters.getX());
        double yFeedback = yController.calculate(currentPose.getY(), desiredState.poseMeters.getY());

        /*
         * Send values to Drivetrain
         */
        ChassisSpeeds adjustedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xFF + xFeedback, yFF + yFeedback, thetaFF,
                currentPose.getRotation());

        Drivetrain.getInstance().setAngleAndDrive(adjustedSpeeds);
    }

    /*
     * Check if the robot has reached the end of the trajectory
     */

    @Override
    public boolean isFinished() {
        return timer.get() >= trajectory.getTotalTimeSeconds();
    }

    /*
     * stop the drivetrain
     */

    @Override
    public void end(boolean interrupted) {
        Drivetrain.getInstance().setAngleAndDrive(new ChassisSpeeds());
    }
}
