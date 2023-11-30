package frc.robot.commands.elevator;

import frc.robot.OI;
import frc.robot.subsystems.AngledElevator;
import harkerrobolib.commands.IndefiniteCommand;

public class ElevatorManual /* extends __ */ {

    /**
     * Constructor for ElevatorManual
     * Associates ElevatorManual with AngledElevator subsystem
     */
    public ElevatorManual() {
        /* CODE HERE */
    }

    /**
     * Move the elevator up or down based on the DPad Button
     */
    public void execute() {
        /** 
         * if DRIVER presses the Up DPad Button, Move the elevator up by 500 steps
         * else if DRIVER presses the Down DPad Button, Move the elevator down by 500 steps
         */

        // Always move to Elevator's Desired position
    }

    /**
     * set power to 0 when end
     * make sure to override the method
     */
}
