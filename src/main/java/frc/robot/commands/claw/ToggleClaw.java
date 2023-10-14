package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.OI;
import frc.robot.subsystems.Claw;

public class ToggleClaw extends InstantCommand{
    public ToggleClaw() {
        addRequirements(Claw.getInstance());
    }

    public void initialize() {    
        if (Claw.getInstance().getClaw().get() == DoubleSolenoid.Value.kForward)
            Claw.getInstance().getClaw().set(DoubleSolenoid.Value.kReverse);
        else
            Claw.getInstance().getClaw().set(DoubleSolenoid.Value.kForward);
    }
}

