package frc.robot.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotMap;
import frc.robot.commands.claw.CloseClaw;
import frc.robot.commands.claw.OpenClaw;
import frc.robot.commands.drivetrain.AlignPitch;
import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.commands.elevator.ZeroElevator;

public class Autons {

    /**
     * topPath : Grab game piece from bot, drop off at node, and move back out of
     * community zone
     */

    public static final SequentialCommandGroup topPath = new SequentialCommandGroup(
            new ZeroElevator(),
            new CloseClaw(),
            new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]),
            new OpenClaw(),
            new MoveToPosition(0),
            new SwervePositionController(
                    Trajectories.topPath,
                    () -> Rotation2d.fromDegrees(180),
                    () -> Rotation2d.fromDegrees(180)));

    /**
     * middlePath : Grab game piece from bot, drop off at node, and move back on
     * chargePad
     */

    public static final SequentialCommandGroup middlePath = new SequentialCommandGroup(
            new ZeroElevator(),
            new CloseClaw(),
            new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]),
            new OpenClaw(),
            new MoveToPosition(0),
            new SwervePositionController(
                    Trajectories.chargePad,
                    () -> Rotation2d.fromDegrees(180),
                    () -> Rotation2d.fromDegrees(180)),
            new AlignPitch());

    /**
     * bottomPath : Grab game piece from bot, drop off at node, and move back out of
     * community zone
     */

    public static final SequentialCommandGroup bottomPath = new SequentialCommandGroup(
            new ZeroElevator(),
            new CloseClaw(),
            new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]),
            new OpenClaw(),
            new MoveToPosition(0),
            new SwervePositionController(
                    Trajectories.bottomPath,
                    () -> Rotation2d.fromDegrees(180),
                    () -> Rotation2d.fromDegrees(180)));

    /**
     * middleAndCross : Grab game piece from bot, drop off at node, and move back
     * over chargePad to cross commmunity line and move back on chargePad (21 point
     * auton)
     */

    public static final SequentialCommandGroup middleAndCross = new SequentialCommandGroup(
            new ZeroElevator(),
            new CloseClaw(),
            new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]),
            new OpenClaw(),
            new MoveToPosition(0)
                    .alongWith(
                            new SwervePositionController(
                                    Trajectories.middleAndCross1,
                                    () -> Rotation2d.fromDegrees(180),
                                    () -> Rotation2d.fromDegrees(180))),
            new SwervePositionController(
                    Trajectories.middleAndCross2,
                    () -> Rotation2d.fromDegrees(180),
                    () -> Rotation2d.fromDegrees(180)),
            new AlignPitch());

    /**
     * noAuton : Grab game piece from bot and drop off at node
     */

    public static final SequentialCommandGroup noAuton = new SequentialCommandGroup(
            new ZeroElevator(),
            new CloseClaw(),
            new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]),
            new OpenClaw(),
            new MoveToPosition(0));
}
