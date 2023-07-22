package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.claw.ToggleClaw;
import frc.robot.commands.drivetrain.AlignPitch;
import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.commands.elevator.ZeroElevator;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.joysticks.XboxGamepad;
import harkerrobolib.util.Constants;

public class OI {
    private static OI instance;

    private XboxGamepad driver;
    private XboxGamepad operator;

    private OI() {
        driver = new XboxGamepad(Constants.DRIVER_ID);
        operator = new XboxGamepad(Constants.OPERATOR_ID);
        initBindings();
    }

    /**
     * @return the driver instance
     */

    public XboxGamepad getDriver() {
        return driver;
    }

    /**
     * @return the operator instance
     */

    public XboxGamepad getOperator() {
        return operator;
    }

    private void initBindings() {
        // set Right DPad Button to toggle claw
        driver.getRightDPadButton().onTrue(new ToggleClaw());

        // X = HP, Y = High, B = Middle, A = Low
        driver.getButtonX().whileTrue(new MoveToPosition(RobotMap.AngledElevator.POSITIONS[3]));
        driver.getButtonY().whileTrue(new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]));
        driver.getButtonB().whileTrue(new MoveToPosition(RobotMap.AngledElevator.POSITIONS[1]));
        driver.getButtonA().whileTrue(new MoveToPosition(RobotMap.AngledElevator.POSITIONS[0]));

        // set LeftBumper Button to Align Pitch
        driver.getLeftBumper().whileTrue(new AlignPitch());

        // set Start Button to Align Yaw (Reset Yaw)
        driver.getButtonStart().onTrue(new InstantCommand(() -> {
            Drivetrain.getInstance().setYaw(0);
        }));

        // set Select Button to Zero Elevator
        driver.getButtonSelect().onTrue(new ZeroElevator());

        // set X button on OPERATOR controller to Zero Elevator
        operator.getButtonX().onTrue(new ZeroElevator());
        // set A button on OPERATOR controller to set Elevator to POSITION LOW
        operator.getButtonA().whileTrue(new MoveToPosition(RobotMap.AngledElevator.POSITIONS[0]));
    }

    /**
     * Singleton Code
     * 
     * @return instance of OI
     */
    public static OI getInstance() {
        if (instance == null)
            instance = new OI();
        return instance;
    }
}