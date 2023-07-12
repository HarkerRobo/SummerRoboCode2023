package frc.robot;

import harkerrobolib.util.Conversions;

public final class RobotMap{
    public static final class SwerveModule{
        public static final double TRANSLATION_GEAR_RATIO = 1.0;
        public static final double WHEEL_DIAMETER = 1.0; 

        public static final String CAN_CHAIN = "rio";
        public static final int [] TRANSLATION_IDS = {0, 0, 0, 0};

        public static final boolean [] TRANSLATION_INVERTS = {false, false, false, false};

        public static final double TRANS_PEAK = 0.0;
        public static final double TRANS_CONTINUOUS  = 0.0;
        public static final double TRANS_PEAK_DUR = 0.0;

        public static final int [] ROTATION_IDS = {0,0,0,0};

        public static final boolean [] ROTATION_INVERTS = {false, false, false, false};
        
        public static final double ROT_PEAK = 0.0;
        public static final double ROT_CONTINUOUS = 0.0;
        public static final double ROT_PEAK_DUR = 0.0;

        public static final double [] CAN_CODER_OFFSETS = {0.0,0.0,0.0,0.0};

        public static final double ROTATION_CONVERSION = Conversions.conversionConstant(Conversions.System.ANGLE, TRANSLATION_GEAR_RATIO, WHEEL_DIAMETER);
        public static final double VELOCITY_CONVERSION = Conversions.conversionConstant(Conversions.System.VELOCITY,TRANSLATION_GEAR_RATIO, WHEEL_DIAMETER);
        public static final double POSITION_CONVERSION = Conversions.conversionConstant(Conversions.System.POSITION,TRANSLATION_GEAR_RATIO, WHEEL_DIAMETER);

    }
}