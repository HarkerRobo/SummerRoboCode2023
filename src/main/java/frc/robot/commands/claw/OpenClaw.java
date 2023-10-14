package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.subsystems.Claw;

public class OpenClaw extends InstantCommand {

    public OpenClaw() {
        addRequirements(Claw.getInstance());
    }

    public void initialize() {
        Claw.getInstance().getClaw().set(DoubleSolenoid.Value.kReverse);
    }
    
}
