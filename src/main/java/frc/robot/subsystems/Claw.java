package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Claw extends SubsystemBase {
    private static Claw instance;

    private DoubleSolenoid claw;

    private Claw() {
        claw = new DoubleSolenoid(RobotMap.Claw.PH_ID, PneumaticsModuleType.REVPH, RobotMap.Claw.CLAW_FORWARD_ID, RobotMap.Claw.CLAW_REVERSE_ID);
        addChild("Claw", claw);
    }

    public DoubleSolenoid getClaw() {
        return claw;
    }

    public static Claw getInstance() {
        if (instance == null)
            instance = new Claw();
    
        return instance;
    }

    // @Override
    // public void initSendable(SendableBuilder builder) {
    //     builder.setSmartDashboardType("Claw");
    //     builder.setActuator(true);
    //     builder.setSafeState(this::releaseClaw);

    //     builder.addBooleanProperty("isOpen", () -> getState(), null);
    // }
}
