package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.AngledElevator;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.Constants;
import harkerrobolib.util.MathUtil;

public class SwerveManual extends IndefiniteCommand {

    private double vx, vy, prevvx, prevvy, omega;

    public SwerveManual() {
        addRequirements(Drivetrain.getInstance());
        prevvx = prevvy = vx = vy = omega = 0;
    }

    public void execute() {
        // set previous x and y velocities
        prevvx = vx;
        prevvy = vy;

        // get x, y, and rotational velocities from joystick
        vx =
            MathUtil.mapJoystickOutput(
                OI.getInstance().getDriver().getLeftY(), Constants.JOYSTICK_DEADBAND);
        vy =
            MathUtil.mapJoystickOutput(
                -OI.getInstance().getDriver().getLeftX(), Constants.JOYSTICK_DEADBAND);
        omega =
            MathUtil.mapJoystickOutput(
                OI.getInstance().getDriver().getRightX(), Constants.JOYSTICK_DEADBAND);


        omega = Drivetrain.getInstance().adjustPigeon(omega);

        // Scaling velocities based on multipliers
        vx = scaleValues(vx, RobotMap.MAX_DRIVING_SPEED) * ((AngledElevator.getInstance().isFarExtended()) ? RobotMap.SwerveManual.CLAMP_MULTIPLIER : RobotMap.SwerveManual.SPEED_MULTIPLIER);
        vy = scaleValues(vy, RobotMap.MAX_DRIVING_SPEED) * ((AngledElevator.getInstance().isFarExtended()) ? RobotMap.SwerveManual.CLAMP_MULTIPLIER : RobotMap.SwerveManual.SPEED_MULTIPLIER);
        omega = scaleValues(omega, RobotMap.MAX_ANGLE_VELOCITY) * ((AngledElevator.getInstance().isFarExtended()) ? RobotMap.SwerveManual.ROT_MULITPLIER : RobotMap.SwerveManual.SPEED_MULTIPLIER);

        // limits acceleration
        vy = limitAcceleration(vy, prevvy);
        vx = limitAcceleration(vx, prevvx);

        
        // aligns to nearest target
        // if (OI.getInstance().getDriver().getRightBumperState()) {
        //     omega = Drivetrain.getInstance().alignToTarget(omega);
        // }

        // sets velocities to zero if robot is not visibly moving
        if (isRobotStill()) {
            vx = 0;
            vy = 0;

            // if rotational velocity is very small
            if (Math.abs(omega) < RobotMap.Drivetrain.MIN_OUTPUT) {
                omega = 0.0001;
            }
        }

        Drivetrain.getInstance()
            .setAngleAndDrive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    vx, vy, -omega, Drivetrain.getInstance().getRotation()));
    }

    /**
     * Limits the drivetrain's acceleration
     * @param value     velocity to correct
     * @param prevValue previous velocity
     * @return          corrected velocity
     */
    private double limitAcceleration(double value, double prevValue) {
        if (Math.abs(value - prevValue) / Constants.ROBOT_LOOP > ((AngledElevator.getInstance().isFarExtended()) ? RobotMap.SwerveManual.MAX_ACCELERATION_EXTENDED : RobotMap.SwerveManual.MAX_ACCELERATION)) {
            value = prevValue + Math.signum(value - prevValue)
                    * ((AngledElevator.getInstance().isFarExtended()) ? RobotMap.SwerveManual.MAX_ACCELERATION_EXTENDED : RobotMap.SwerveManual.MAX_ACCELERATION)
                    * Constants.ROBOT_LOOP;
            // previous velocity + direction of movement (+/-) * acceleration * time (a=v/t)
        }

        return value;
    }

    /**
     * Scales the velocities by their given multiplier
     * @param value         velocity to scale
     * @param scaleFactor   multiplier
     * @return              scaled velocity
     */
    private double scaleValues(double value, double scaleFactor) {
        return value * scaleFactor;
    }

    /**
     * @return if the robot is moving slow enough for it to be considered still
     */
    private boolean isRobotStill() {
        return Math.sqrt(vx * vx + vy * vy) < RobotMap.Drivetrain.MIN_OUTPUT;
    }

    /**
     * Sets the x, y, and rotational velocities to 0.
     */
    public void end(boolean interrupted) {
        Drivetrain.getInstance().setAngleAndDrive(new ChassisSpeeds());
    }
}