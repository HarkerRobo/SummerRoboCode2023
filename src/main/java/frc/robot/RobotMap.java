package frc.robot;

import harkerrobolib.util.Conversions;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public final class RobotMap {
    public static final class Field {
        // field dimensions
        public static final double fieldLength = Units.inchesToMeters(651.25);
        public static final double fieldWidth = Units.inchesToMeters(315.5);
        public static final Field2d FIELD = new Field2d();

        // indexes for game pieces and field
        public static final int TAPE_INDEX = 0;
        public static final int CONE_INDEX = 1;
    }

    // Robot Constants
    public static final double MAX_DRIVING_SPEED = 4.0; // m/s
    public static final double MAX_ANGLE_VELOCITY = Math.PI;
    public static final double MAX_ANGLE_ACCELERATION = MAX_ANGLE_VELOCITY / 2;

    public static final String CAN_CHAIN = "rio";

    // Robot Dimensions
    public static final double ROBOT_LENGTH = Units.inchesToMeters(30);
    public static final double ROBOT_WIDTH = Units.inchesToMeters(28);

    public static final class Drivetrain {
        // Pigeon ID
        public static final int PIGEON_ID = 1;

        public static final double MIN_OUTPUT = 0.01;
    }
  
   public static final class SwerveModule {
        //gear ratio 
        public static final double TRANSLATION_GEAR_RATIO = 1.0;
        //diameter of the wheel
        public static final double WHEEL_DIAMETER = 1.0; 

        //can chain 
        public static final String CAN_CHAIN = "rio";
        //id of translation motors 
        public static final int [] TRANSLATION_IDS = {0, 0, 0, 0};

        //translation motors inverted
        public static final boolean [] TRANSLATION_INVERTS = {false, false, false, false};

        //current limit constants for translation motors 
        public static final double TRANS_PEAK = 0.0;
        public static final double TRANS_CONTINUOUS  = 0.0;
        public static final double TRANS_PEAK_DUR = 0.0;

        //ids for rotation motors
        public static final int [] ROTATION_IDS = {0,0,0,0};

        //rotation motors inverted
        public static final boolean [] ROTATION_INVERTS = {false, false, false, false};
        
       //current limit constants for rotation motors  
        public static final double ROT_PEAK = 0.0;
        public static final double ROT_CONTINUOUS = 0.0;
        public static final double ROT_PEAK_DUR = 0.0;

        //offsets of cancoders of each swerve module 
        public static final double [] CAN_CODER_OFFSETS = {0.0,0.0,0.0,0.0};

        //conversions from native units
        public static final double ROTATION_CONVERSION = Conversions.conversionConstant(Conversions.System.ANGLE, TRANSLATION_GEAR_RATIO, WHEEL_DIAMETER);
        public static final double VELOCITY_CONVERSION = Conversions.conversionConstant(Conversions.System.VELOCITY,TRANSLATION_GEAR_RATIO, WHEEL_DIAMETER);
        public static final double POSITION_CONVERSION = Conversions.conversionConstant(Conversions.System.POSITION,TRANSLATION_GEAR_RATIO, WHEEL_DIAMETER);
    }

    public static final class Claw {
        public static final int CLAW_FORWARD_ID = 0;
        public static final int CLAW_REVERSE_ID = 0;
    }

    public static final class AngledElevator {
        public static final double kP = 0.13; // Based on RoboCode2023; TUNE
        public static final double kG = 0.09; // Based on RoboCode2023; TUNE

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

        // POSITIONS:  Low, Middle, High, Human Player (HP)
        public static double[] POSITIONS = {
            10018, 27000, 39500, 27500
        };

        // HORIZONTAL Offsets: Middle, High, Human Player (HP)
        public static double[] HORIZONTAL_OFFSET = {
            10018, 27000, 39500
        };
    }
}
