package frc.robot.util;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.RobotMap;
import harkerrobolib.util.Constants;

public class MotorVelocitySystem {

    private LinearSystem<N1, N1, N1> plant;
    private KalmanFilter<N1, N1, N1> observer;
    private LinearQuadraticRegulator<N1, N1, N1> controller;
    private LinearSystemLoop<N1, N1, N1> loop;

    private double kS; // Feedforward constant

    /**
     * Constructor to create a MotorVelocitySystem object.
     * 
     * @param kS    The feedforward constant used to apply a feedforward term to the
     *              control input.
     * @param kV    The velocity gain used to create a linear system model for the
     *              motor velocity.
     * @param kA    The acceleration gain used to create a linear system model for
     *              the motor velocity.
     * @param error The error vector used in the Linear-Quadratic Regulator (LQR)
     *              for controller design.
     */
    public MotorVelocitySystem(double kS, double kV, double kA, double error) {
        this.kS = kS;

        /** 
         * Create a linear system model for the motor velocity based on the given kV and
         * kA gains
         * 
         * A mathematical model that predicts how the motor's velocity responds to
         * different inputs, like voltage or power, in a straightforward manner.
         */
        plant = LinearSystemId.identifyVelocitySystem(kV, kA);

        // Create a Kalman Filter
        /**
         * An algorithm that estimates the motor's true velocity, taking into account
         * measurement uncertainties and noise from the encoders, providing a more
         * accurate estimation.
         */
        observer = new KalmanFilter<>(Nat.N1(), Nat.N1(), plant, RobotMap.MotorVelocitySystem.MODEL_STD_DEV,
                RobotMap.MotorVelocitySystem.ENCODER_STD_DEV, Constants.ROBOT_LOOP);

        // Create a Linear-Quadratic Regulator (LQR)
        /**
         * A control strategy that calculates the optimal voltage input to achieve the
         * desired motor speed while minimizing the difference between the desired and
         * actual speeds, ensuring stable and efficient motor control.
         */
        controller = new LinearQuadraticRegulator<>(plant, VecBuilder.fill(error), RobotMap.MotorVelocitySystem.RELMS,
                Constants.ROBOT_LOOP);

        // Create a Linear System Loop to encapsulate the control loop
        loop = new LinearSystemLoop<>(plant, controller, observer, Constants.MAX_VOLTAGE, Constants.ROBOT_LOOP);
    }

    /**
     * Calculates the voltage needed to achieve the desired speed for the motor.
     * 
     * @param desiredSpeed The desired motor speed in meters per second (m/s).
     * @param currentSpeed The current motor speed in meters per second (m/s).
     * @return The voltage needed to achieve the desired speed.
     */
    public double getVoltage(double desiredSpeed, double currentSpeed) {
        // Set the next reference input to the desired speed for the control loop
        loop.setNextR(VecBuilder.fill(desiredSpeed));

        // Correct the state estimate using the current speed measurement
        loop.correct(VecBuilder.fill(currentSpeed));

        // Predict the next state using the current control loop settings
        loop.predict(Constants.ROBOT_LOOP);

        // Calculate the next voltage to be applied using the control input and the
        // feedforward term
        // The feedforward term helps to compensate for steady-state errors and improve
        // system response
        double nextVoltage = loop.getU(0) + kS * Math.signum(desiredSpeed - currentSpeed);
        return nextVoltage;
    }
}