package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.RobotMap.SwervePositionController;
import frc.robot.subsystems.Drivetrain;

public class Telemetry {
    NetworkTable table; 
    NetworkTableInstance inst;
    public Telemetry(){
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("datatable");
    }

    public void putButtons() {
        NetworkTable buttons = table.getSubTable("Button State");

        NetworkTableEntry rightDPad = buttons.getEntry("Right DPad State");
        
        boolean state = (OI.getInstance().getOperator().getRightDPadButtonState());
        if (state) {
            rightDPad.setNumber(1);
        }
        else {
            rightDPad.setNumber(0);
        }
    }

    // public void putSwerveModule(){
    //     NetworkTable swervemodule = table.getSubTable("Swerve Module");

    //     NetworkTableEntry rotationkP = swervemodule.getEntry("Rotation kP");
    //     rotationkP.setDouble(RobotMap.SwerveModule.ROTATION_KP);

    //     NetworkTableEntry translationkP = swervemodule.getEntry("Translation kP");
    //     translationkP.setDouble(RobotMap.SwerveModule.TRANSLATION_KP);

    //     NetworkTableEntry translationkI = swervemodule.getEntry("Transltion kI");
    //     translationkI.setDouble(RobotMap.SwerveModule.TRANSLATION_KI);

    //     NetworkTableEntry translationkD = swervemodule.getEntry("Translation kD");
    //     translationkD.setDouble(RobotMap.SwerveModule.TRANSLATION_KD);

    //     NetworkTableEntry translationkS = swervemodule.getEntry("Translation kS");
    //     translationkS.setDouble(RobotMap.SwerveModule.TRANSLATION_KS);

    //     NetworkTableEntry translationkV = swervemodule.getEntry("Transltion kV");
    //     translationkV.setDouble(RobotMap.SwerveModule.TRANSLATION_KV);

    //     NetworkTableEntry translationkA = swervemodule.getEntry("Translation kA");
    //     translationkA.setDouble(RobotMap.SwerveModule.TRANSLATION_KA);
    // }
    // public void putDrivetrain(){
    //     NetworkTable drivetrain = table.getSubTable("Drivetrain");
    //     NetworkTableEntry kP = drivetrain.getEntry("Pigeon kP");
    //     kP.setDouble(RobotMap.Drivetrain.PIGEON_kP);
    // }
    // public void putElevator(){
    //     NetworkTable elevator = table.getSubTable("Elevator");

    //     NetworkTableEntry kP = elevator.getEntry("kP");
    //     kP.setDouble(RobotMap.AngledElevator.kP);

    //     NetworkTableEntry kG = elevator.getEntry("kG");
    //     kG.setDouble(RobotMap.AngledElevator.kG);
    // }

    // public void putAlignPitch(){
    //     NetworkTable alignpitch = table.getSubTable("Align Pitch");

    //     NetworkTableEntry kP = alignpitch.getEntry("kP");
    //     kP.setDouble(RobotMap.AlignPitch.kP);

    //     NetworkTableEntry kI = alignpitch.getEntry("kI");
    //     kI.setDouble(RobotMap.AlignPitch.kI);

    //     NetworkTableEntry kD = alignpitch.getEntry("kD");
    //     kD.setDouble(RobotMap.AlignPitch.kD);
    // }

    // public void putSwervePositionController(){
    //     NetworkTable swervepositioncontroller = table.getSubTable("SwervePositionController");

    //     NetworkTableEntry X_kP = swervepositioncontroller.getEntry("X_kP");
    //     X_kP.setDouble(RobotMap.SwervePositionController.X_kP);

    //     NetworkTableEntry X_kI = swervepositioncontroller.getEntry("X_kI");
    //     X_kI.setDouble(RobotMap.SwervePositionController.X_kI);

    //     NetworkTableEntry X_kD = swervepositioncontroller.getEntry("X_kD");
    //     X_kD.setDouble(RobotMap.SwervePositionController.X_kD);

    //     NetworkTableEntry Y_kP = swervepositioncontroller.getEntry("Y_kP");
    //     Y_kP.setDouble(RobotMap.SwervePositionController.Y_kP);

    //     NetworkTableEntry Y_kI = swervepositioncontroller.getEntry("Y_kI");
    //     Y_kI.setDouble(RobotMap.SwervePositionController.Y_kI);

    //     NetworkTableEntry Y_kD = swervepositioncontroller.getEntry("Y_kD");
    //     Y_kD.setDouble(RobotMap.SwervePositionController.Y_kD);

    //     NetworkTableEntry THETA_kP = swervepositioncontroller.getEntry("THETA_kP");
    //     THETA_kP.setDouble(RobotMap.SwervePositionController.THETA_kP);

    //     NetworkTableEntry THETA_kI = swervepositioncontroller.getEntry("THETA_kI");
    //     THETA_kI.setDouble(RobotMap.SwervePositionController.THETA_kI);

    //     NetworkTableEntry THETA_kD = swervepositioncontroller.getEntry("THETA_kD");
    //     THETA_kD.setDouble(RobotMap.SwervePositionController.THETA_kD);
    // }
}