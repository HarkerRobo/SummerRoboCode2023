package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.SwerveModule;

public class Drivetrain extends SubsystemBase {
    // instance variable for singleton

    // array of swerve modules

    // pigeon

    // previous heading

    // debouncer

    // kinematics
    // converts chassis speeds (x, y, theta) to module states (speed, angle)

    // profiled PID controller; max velocity is 4, max acceleration is 3.5

    // Estimates the robot's pose through encoder (state) and vision measurements;

    // Standard deviations of pose estimate (x, y, heading)

    private Drivetrain() {
        // initialize swerve modules

        // initialize pigeon and call the initPigeon method

        // kinematics - initialize locations of swerve modules relative to robot (fl, fr, bl, br)

        // sets how much error to allow on theta controller; will be in RobotMap

        // initial pose (holds the x, y, heading)

        // initialize pose estimator

        // initialize debouncer with a 0.4 second delay and a rising debounce type
    }

    /*
     * Initialize pigeon values
     * Configs factory default
     * Configs mount pose yaw, pitch, and roll
     * Sets yaw to 0
     * Configs compass to false
     */
    private void initPigeon() {
        return;
    }

    /**
     * Updates previous heading to the current heading or continues in the
     * same direction (omega becomes adjusted by the prevous heading)
     * @param omega rotational speed
     * @return      adjusted rotational speed
     */
    public double adjustPigeon(double omega) {
        if (debouncer.calculate(Math.abs(omega) <= RobotMap.Drivetrain.MIN_OUTPUT)) {
            omega = -RobotMap.Drivetrain.PIGEON_kP * (prevHeading - getHeading());
        }
        else {
            prevHeading = getHeading();
        }
    
        return omega;
    }

    /**
     * Returns yaw of pigeon in degrees (heading of robot)
     * @return yaw of pigeon in degrees
     */
    public double getHeading() {
        return 0;
    }

    /**
     * @return pitch of pigeon in degrees
     */
    public double getPitch() {
        return 0;
    }

    /**
     * @return roll of pigeon in degrees
     */
    public double getRoll() {
        return 0;
    }

    /**
     * @return heading of pigeon as a Rotation2d
     */
    public Rotation2d getRotation() {
        return new Rotation2d();
    }

    /**
     * @return the positions of the swerve modules in an array
     */
    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {};
    }

    /**
     * Sets the initial pose of the drivetrain
     * zeros the translation of each swerve module
     * sets the yaw of the pigeon to the given angle (in degrees)
     * updates the pose estimator (using resetPosition and the the current rotation, module positions, and pose)
     * @param pose      intial Pose2d of drivetrain
     */
    public void setPose(Pose2d pose) {
        return;
    }

    /**
     * Sets the yaw of the pigeon to the given angle
     * Updates previous heading to the current heading
     * @param yaw   angle in degrees (double)
     */
    public void setYaw() {
        return;
    }

    /**
     * Updates previous heading
     * @param prev new heading
     */
    public void setPreviousHeading(double prev) {
        return;
    }

    /**
     * Uses CameraPoseEstimation to align to the detected target and returns
     * the needed angle to adjust to the target
     * @param omega     rotational speed
     * @return          adjusted rotational speed
     */
    // public double alignToTarget(double omega) {
    //     var result = CameraPoseEstimation.getInstance().getCamera().getLatestResult();
    //     if (result.hasTargets()) {
    //         omega =
    //             -thetaController.calculate(result.getBestTarget().getYaw() - RobotMap.Drivetrain.OFFSET);
    //         setPreviousHeading(getHeading());
    //     }
    //     return omega;
    //   }

    /**
     * @return kinematics of swerve drive
     */
    public SwerveDriveKinematics getKinematics() {
        return new SwerveDriveKinematics(null, null, null, null);
    }

    /**
     * Singleton code
     * @return instance of Drivetrain
     */

    /**
     * Converts chassis speeds to individual swerve module 
     * states and sets the angle and drive for them
     * @param chassis       chassis speeds to convert
     */
    public void setAngleAndDrive(ChassisSpeeds chassis) {
        return;
    }

    /**
     * Called every loop, feeds newest encoder readings to estimator
     * gets the current rotation and module positions and updates the pose
     */
    public void updatePose() {
        return;
    }

    /**
     * @return the estimated pose aas a Pose2d
     */
    public Pose2d getPoseEstimatorPose2d() {
        return new Pose2d();
    }

    /**
     * updates the pose of the drivetrain
     * make sure to override the method
     */
    public void periodic() {
        return;
    }
}
