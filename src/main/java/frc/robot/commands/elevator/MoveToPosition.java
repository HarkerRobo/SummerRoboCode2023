package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AngledElevator;

public class MoveToPosition extends CommandBase {

    private double position;
    private final Timer timer = new Timer();

    public MoveToPosition(double position) {
        this.position = position;
        addRequirements(AngledElevator.getInstance());
    }

    /**
     * Resets the timer and starts it (stopwatch)
     */
    public void initialize() {
        timer.reset();
        timer.start();
    }

    /**
     * Moves the elevator to a set position
     */
    public void execute() {
        AngledElevator.getInstance().moveToPosition(position);
    
    }

    /**
     * if the Elevator is at the right position or the timer is above 2.5 seconds, return true
     */

    public boolean isFinished() {
        return (AngledElevator.getInstance().checkExtend(position) || timer.get() > 2.5);
    }

    /**
     * Keep the elevator at the set position
     */
    public void end(boolean interrupted) {
        AngledElevator.getInstance().setDesiredPosition(position);
        AngledElevator.getInstance().setElevatorPower(0);
    }
}