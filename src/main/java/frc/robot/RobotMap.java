package frc.robot;

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

    // Robot Dimensions
    public static final double ROBOT_LENGTH = Units.inchesToMeters(30);
    public static final double ROBOT_WIDTH = Units.inchesToMeters(28);

    public static final class Drivetrain {
        // Pigeon ID
        public static final int PIGEON_ID = 1;

        public static final double MIN_OUTPUT = 0.01;
    }
}
