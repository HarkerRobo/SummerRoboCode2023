package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Claw /* extends something */ extends SubsystemBase {
    // instance variable for singleton

    // Double Solenoid variable

    /**
     * constructor for the claw
     * instanciates the double solenoid variable
     * 
     * module type is REVPH
     * use ids from RobotMap
     */
    private Claw() {
        /* CODE HERE */
    }

    /**
     * Getter for the claw double solenoid variable
     * @return the variable
     */
    public DoubleSolenoid getClaw() {
        /* CODE HERE */
    }

    /**
     * SINGLETON Code Here
     */
}
