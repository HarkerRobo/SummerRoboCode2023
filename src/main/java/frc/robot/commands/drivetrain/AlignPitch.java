package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;

public class AlignPitch extends CommandBase {

    private static ProfiledPIDController pitchController =
        new ProfiledPIDController(
            RobotMap.AlignPitch.kP, RobotMap.AlignPitch.kI, RobotMap.AlignPitch.kD, new Constraints(RobotMap.MAX_DRIVING_SPEED, RobotMap.MAX_DRIVING_SPEED / 2));

    public AlignPitch() {
        SmartDashboard.putNumber("AlignPitchkP",RobotMap.AlignPitch.kP);
        addRequirements(Drivetrain.getInstance());
    }

    /**
     * Initializes the pitch controller by reseting its values and setting
     * its tolerance and setpoint
     */
    public void initialize() {
        pitchController.reset(0);
        pitchController.setTolerance(RobotMap.AlignPitch.MAX_ERROR_PITCH);
        pitchController.setGoal(RobotMap.AlignPitch.SETPOINT);
    }

    /**
     * Feeds the error into the pitch controller and moves forward by the
     * calculated velocity
     */
    public void execute() {
        double error = RobotMap.AlignPitch.SETPOINT - Drivetrain.getInstance().getRoll();
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
        return pitchController.atSetpoint()||OI.getInstance().getDriver().getUpDPadButtonState();
    }

    /**
     * Sets the x, y, and rotational velocities to 0.
     */
    public void end(boolean interrupted) {
        Drivetrain.getInstance().setAngleAndDrive(new ChassisSpeeds());
    }
}
