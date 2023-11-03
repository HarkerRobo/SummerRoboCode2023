package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.AngledElevator;

public class SoftZero extends CommandBase {

    public SoftZero() {
        addRequirements(AngledElevator.getInstance());
    }

    /**
     * Slowly move the elevator down
     */

    public void execute() {
        AngledElevator.getInstance().setElevatorPower(-0.02);
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
        if(!interrupted) {
            AngledElevator.getInstance().resetEncoders();
            AngledElevator.getInstance().setDesiredPosition(0);
            AngledElevator.getInstance().setElevatorPower(0);
        }
    }
}