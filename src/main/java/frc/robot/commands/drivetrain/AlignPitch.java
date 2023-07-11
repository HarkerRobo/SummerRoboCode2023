package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;

public class AlignPitch extends CommandBase {
    // PID values for the pitch controller
    public static final double kP = 0.05;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // Pitch value to reach (0 since the robot should be flat)
    public static final double SETPOINT = 0;

    // Error tolerance
    public static final double MAX_ERROR_PITCH = 0.01;

    private static ProfiledPIDController pitchController =
        new ProfiledPIDController(
            kP, kI, kD, new Constraints(RobotMap.MAX_DRIVING_SPEED, RobotMap.MAX_DRIVING_SPEED / 2));

    public AlignPitch() {
        addRequirements(Drivetrain.getInstance());
    }

    /**
     * Initializes the pitch controller by reseting its values and setting
     * its tolerance and setpoint
     */
    public void initialize() {
        pitchController.reset(0);
        pitchController.setTolerance(MAX_ERROR_PITCH);
        pitchController.setGoal(SETPOINT);
    }

    /**
     * Feeds the error into the pitch controller and moves forward by the
     * calculated velocity
     */
    public void execute() {
        double error = SETPOINT - Drivetrain.getInstance().getRoll();
        double forwardAmount = pitchController.calculate(error);
        double omega = Drivetrain.getInstance().adjustPigeon(0);
        ChassisSpeeds speeds = new ChassisSpeeds(forwardAmount, 0, 0);
        Drivetrain.getInstance().setAngleAndDrive(speeds);
    }

    /**
     * Stops command when the pitch value is within the error tolerance
     * of the setpoint
     */
    @Override
    public boolean isFinished() {
        return pitchController.atSetpoint();
    }

    /**
     * Sets the x, y, and rotational velocities to 0.
     */
    public void end(boolean interrupted) {
        Drivetrain.getInstance().setAngleAndDrive(new ChassisSpeeds());
    }
}
