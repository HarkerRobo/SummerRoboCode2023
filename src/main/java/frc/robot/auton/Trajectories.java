package frc.robot.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotMap;

import java.util.ArrayList;
import java.util.List;

public class Trajectories {
    /**
     * chargePad : Move to charge pad
     */

    public static Trajectory chargePad = generateTrajectory(
            List.of(
                    new Pose2d(1.91, 2.75, Rotation2d.fromDegrees(180)),
                    new Pose2d(3.48, 2.75, Rotation2d.fromDegrees(180))),
            2.0,
            1.0,
            0.0,
            0.0,
            true);

    /**
     * topPath : Grab game piece from bot, drop off at node, and move back out of
     * community zone
     */

    public static Trajectory topPath = generateTrajectory(
            List.of(
                    new Pose2d(1.91, 4.44, Rotation2d.fromDegrees(180)),
                    new Pose2d(4.08, 4.80, Rotation2d.fromDegrees(180)),
                    new Pose2d(5.96, 5.53, Rotation2d.fromDegrees(180))),
            2.0,
            1.0,
            0,
            0,
            true);

    /**
     * bottomPath : Grab game piece from bot, drop off at node, and move back out of
     * community zone
     */

    public static Trajectory bottomPath = generateTrajectory(
            List.of(
                    new Pose2d(1.91, 0.95, Rotation2d.fromDegrees(180)),
                    new Pose2d(7, 0.55, Rotation2d.fromDegrees(180))),
            3.0,
            1.5,
            0.0,
            0.0,
            true);

    /**
     * middleAndCross1 : Go back and cross community
     */

    public static Trajectory middleAndCross1 = generateTrajectory(
            List.of(
                    new Pose2d(1.91, 2.75, Rotation2d.fromDegrees(180)),
                    new Pose2d(6.5, 2.75, Rotation2d.fromDegrees(180))),
            2.5,
            1.0,
            0.0,
            0.0,
            true);

    /**
     * middleAndCross2 : Go back on charge pad and align
     */

    public static Trajectory middleAndCross2 = generateTrajectory(
            List.of(
                    new Pose2d(6.5, 2.75, Rotation2d.fromDegrees(180)),
                    new Pose2d(4.00, 2.75, Rotation2d.fromDegrees(180))),
            3.0,
            1.5,
            0.0,
            0.0,
            false);

    /**
     * generates a Trajectory given a list of Pose2d points, max velocity, max
     * acceleration, start velocity, and end velocity, and if flipped due to
     * alliance
     */

    public static Trajectory generateTrajectory(
            List<Pose2d> waypoints,
            double maxVelocity,
            double maxAcceleration,
            double startVelocity,
            double endVelocity,
            boolean reversed,
            TrajectoryConstraint... constraints) {
        TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);

        for (TrajectoryConstraint constraint : constraints) {
            config.addConstraint(constraint);
        }

        config.setStartVelocity(startVelocity);
        config.setEndVelocity(endVelocity);
        config.setReversed(reversed);

        // interiorPoints - points between the two end points along auton path
        List<Translation2d> interiorPoints = new ArrayList<Translation2d>();
        for (int i = 1; i < waypoints.size() - 1; i++) {
            interiorPoints.add(waypoints.get(i).getTranslation());
        }

        return TrajectoryGenerator.generateTrajectory(
                waypoints.get(0), interiorPoints, waypoints.get(waypoints.size() - 1), config);
    }

    /**
     * Flips the Translation2d based on current alliance
     */

    public static Translation2d apply(Translation2d translation2d) {
        return (isFlipped())
                ? new Translation2d(RobotMap.Field.fieldLength - translation2d.getX(),
                        translation2d.getY())
                : translation2d;
    }

    /**
     * Flips the Rotation2d based on current alliance
     */

    public static Rotation2d apply(Rotation2d rotation2d) {
        return (isFlipped()) ? new Rotation2d(-rotation2d.getCos(), rotation2d.getSin()) : rotation2d;
    }

    /**
     * Flips the Pose2d based on current alliance
     */

    public static Pose2d apply(Pose2d pose2d) {
        return (isFlipped())
                ? new Pose2d(RobotMap.Field.fieldLength - pose2d.getX(), pose2d.getY(),
                        new Rotation2d(-pose2d.getRotation().getCos(), pose2d.getRotation().getSin()))
                : pose2d;
    }

    /**
     * Flips the Trajectory.State based on current alliance
     */

    public static Trajectory.State apply(Trajectory.State state) {
        return (isFlipped()) ? new Trajectory.State(
                state.timeSeconds,
                state.velocityMetersPerSecond,
                state.accelerationMetersPerSecondSq,
                new Pose2d(
                        RobotMap.Field.fieldLength - state.poseMeters.getX(),
                        state.poseMeters.getY(),
                        new Rotation2d(
                                -state.poseMeters.getRotation().getCos(),
                                state.poseMeters.getRotation().getSin())),
                -state.curvatureRadPerMeter) : state;
    }

    /**
     * if Alliance is red, flip the trajectory
     * field flipped along y-axis
     */
    public static boolean isFlipped() {
        return DriverStation.getAlliance() == Alliance.Red;
    }
}