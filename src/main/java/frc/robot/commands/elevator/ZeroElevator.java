package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AngledElevator;

public class ZeroElevator extends CommandBase {

    public static final double ZERO_SPEED = -0.45;

    public ZeroElevator() {
        addRequirements(AngledElevator.getInstance());
    }

    /**
     * Slowly move the elevator down
     */

    public void execute() {
        AngledElevator.getInstance().setElevatorPower(ZERO_SPEED);
    }

    /**
     * if Elevator is at "Zero", stop Power
     */

    @Override
    public boolean isFinished() {
        return AngledElevator.getInstance().extensionStop();
    }

    /**
     * End: Reset Encoders and Move Elevator to lowest position (ground level)
     */

    public void end(boolean interrupted) {
        AngledElevator.getInstance().resetEncoders();
        AngledElevator.getInstance().setDesiredPosition(0);
    }
}