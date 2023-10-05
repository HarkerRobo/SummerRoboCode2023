package frc.robot;

import harkerrobolib.util.Constants;
import harkerrobolib.util.Conversions;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public final class RobotMap {

    // Global Robot Constants
    public static final double MAX_DRIVING_SPEED = 4.0; // m/s
    public static final double MAX_ANGLE_VELOCITY = Math.PI;
    public static final double MAX_ANGLE_ACCELERATION = MAX_ANGLE_VELOCITY / 2;

    public static final String CAN_CHAIN = "rio";

    // Robot Dimensions
    public static final double ROBOT_LENGTH = Units.inchesToMeters(30);
    public static final double ROBOT_WIDTH = Units.inchesToMeters(28);

    public static final class Field {
        // field dimensions
        public static final double fieldLength = Units.inchesToMeters(651.25);
        public static final double fieldWidth = Units.inchesToMeters(315.5);
        public static final Field2d FIELD = new Field2d();

        // indexes for game pieces and field
        public static final int TAPE_INDEX = 0;
        public static final int CONE_INDEX = 1;
    }

    public static final class Drivetrain {
        // Pigeon ID
        public static final int PIGEON_ID = 1;

        public static final double PIGEON_kP = 0.067;

        public static final double MIN_OUTPUT = 0.05;

        public static final double MAX_ERROR_YAW = 0.5;
        public static final double OFFSET = 9.5;

        // Profiled PID for theta (turning) control
        public static final double THETA_P = 0.118;
        public static final double THETA_I = 0.0;
        public static final double THETA_D = 0.0;
    }

    public static final class SwerveModule {
        // id of translation motors
        public static final int[] TRANSLATION_IDS = {1, 2, 3, 4};

        // translation motors inverted
        public static final boolean[] TRANSLATION_INVERTS = {false, false, false, false};

        // ids for rotation motors
        public static final int[] ROTATION_IDS = {5, 6, 7, 8};

        // rotation motors inverted
        public static final boolean[] ROTATION_INVERTS = {false, false, false, false};

        // cancoder ids
        public static final int[] CAN_CODER_ID = {9, 10, 11, 12};

        // offsets of cancoders of each swerve module
        public static final double[] CAN_CODER_OFFSETS = {323.613 - 180, 244.951 - 180, 266.572 + 180, 91.406};

        // current limit constants for translation motors
        public static final double TRANS_PEAK = 30;
        public static final double TRANS_CONTINUOUS = 55;
        public static final double TRANS_PEAK_DUR = 0.1;

        // current limit constants for rotation motors
        public static final double ROT_PEAK = 25;
        public static final double ROT_CONTINUOUS = 40;
        public static final double ROT_PEAK_DUR = 0.1;

        // velocity measurement window for translation motors
        public static final int VELOCITY_WINDOW = 32;

        // gear ratios
        public static final double TRANSLATION_GEAR_RATIO = 6.75;
        public static final double ROTATION_GEAR_RATIO = 12.8;
        // diameter of the wheel
        public static final double WHEEL_DIAMETER = 4.0; // inches

        // conversions from native units
        public static final double ROTATION_CONVERSION = Conversions.conversionConstant(Conversions.System.ANGLE,
                ROTATION_GEAR_RATIO, WHEEL_DIAMETER);
        public static final double VELOCITY_CONVERSION = Conversions.conversionConstant(Conversions.System.VELOCITY,
                TRANSLATION_GEAR_RATIO, WHEEL_DIAMETER);
        public static final double POSITION_CONVERSION = Conversions.conversionConstant(Conversions.System.POSITION,
                TRANSLATION_GEAR_RATIO, WHEEL_DIAMETER);

        // rotation kP
        public static final double ROTATION_KP = 0.2;

        // Translation FF Values
        public static final double TRANSLATION_KS = 0.02569;
        public static final double TRANSLATION_KV = 1.954584;
        public static final double TRANSLATION_KA = 0.21522;

        // pid
        public static final double TRANSLATION_KP = 0.076;
        public static final double TRANSLATION_KI = 0.00;
        public static final double TRANSLATION_KD = 0.00; //0.9;
        // ; //0.7;
    }

    public static final class AlignPitch {
        // PID values for the pitch controller
        public static final double kP = 0.014; // no bumpers - 0.0187, with - 0.017
        public static final double kI = 0.0;
        public static final double kD = 0;

        // Pitch value to reach (0 since the robot should be flat)
        public static final double SETPOINT = 0;

        // Error tolerance
        public static final double MAX_ERROR_PITCH = 0.01;
    }

    public static final class SwerveManual {
        // Speed multipliers
        public static final double SPEED_MULTIPLIER = 0.9;
        public static final double ROT_MULITPLIER = 0.9;
        public static final double CLAMP_MULTIPLIER = 0.7;
        public static final double MAX_ACCELERATION = 15;
        public static final double MAX_ACCELERATION_EXTENDED = 6;
    }

    // CLAW
    public static final class Claw {
        public static final int CLAW_FORWARD_ID = 0;
        public static final int CLAW_REVERSE_ID = 1;
    }

    // ELEVATOR
    
    public static final class AngledElevator {
        public static final double kP = 0.12; // Based on RoboCode2023; TUNE
        public static final double kG = 0.087; // Based on RoboCode2023; TUNE

        public static final double MAX_ERROR = 100;

        public static final double CRUISE_VELOCITY = 7447; // NEEDS TO BE TUNED (based on RoboCode2023)
        public static final double CRUISE_ACCELERATION = 4447; // NEEDS TO BE TUNED (based on RoboCode2023)

        public static final int MASTER_ID = 15; // Left Motor
        public static final int FOLLOWER_ID = 14; // Right Motor
        public static final int LIMIT_SWITCH_ID = 0;

        public static final boolean MASTER_INVERTED = false; // TODO
        public static final boolean FOLLOWER_INVERTED = false; // TODO

        // Thresholds for soft-limits
        public static final double FORWARD_LIMIT = 40000;
        public static final double REVERSE_LIMIT = 0;

        // Time for the motor to go from neutral to full
        public static final double RAMP_TIME = 0.01;

        // POSITIONS: Low, Middle, High, Human Player (HP)
        public static double[] POSITIONS = {
                10018, 27000, 39500, 28500
        };

        // HORIZONTAL Offsets: Middle, High, Human Player (HP)
        public static double[] HORIZONTAL_OFFSET = {
                10018, 27000, 39500
        };
    }

    public static final class ZeroElevator {
        public static final double ZERO_SPEED = -0.37;
    }

    // AUTON CONSTANTS
    public static final class SwervePositionController {
        /**
         * PID values for X, Y, and Rotation (THETA)
         */

        public static double X_kP = 3.0; // TUNE (based on RoboCode2023 values)
        public static double X_kI = 0.0;
        public static double X_kD = 0.0;

        public static double Y_kP = 2.5; // TUNE (based on RoboCode2023 values)
        public static double Y_kI = 0.0;
        public static double Y_kD = 0.0;

        public static double THETA_kP = 1.0; // TUNE (based on RoboCode2023 values)
        public static double THETA_kI = 0.0;
        public static double THETA_kD = 0.0;
    }
}
