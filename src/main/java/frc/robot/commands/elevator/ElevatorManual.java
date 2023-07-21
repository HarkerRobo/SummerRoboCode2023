package frc.robot.commands.elevator;

import frc.robot.OI;
import frc.robot.subsystems.AngledElevator;
import harkerrobolib.commands.IndefiniteCommand;

public class ElevatorManual extends IndefiniteCommand {

    public ElevatorManual() {
        addRequirements(AngledElevator.getInstance());
    }
    
    public void execute() {
        /** 
         * if DRIVER presses the Up DPad Button, Move the elevator up by 500 steps
         * else if DRIVER presses the Down DPad Button, Move the elevator down by 500 steps
         */
        
        if (OI.getInstance().getDriver().getUpDPadButtonState()) {
            AngledElevator.getInstance().setDesiredPosition(AngledElevator.getInstance().getPosition() + 500);
        }
        else if (OI.getInstance().getDriver().getDownDPadButtonState())  {
            AngledElevator.getInstance().setDesiredPosition(AngledElevator.getInstance().getPosition() - 500);
        }

        // Always move to Elevator's Desired position
        AngledElevator.getInstance().moveToPosition(AngledElevator.getInstance().getDesiredPosition());
    }

    /**
     * set power to 0 when end
     */

    @Override
    public void end(boolean interrupted) {
        AngledElevator.getInstance().setElevatorPower(0);
    }
}
