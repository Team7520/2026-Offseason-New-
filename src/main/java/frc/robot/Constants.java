package frc.robot;

public class Constants {
    public static class IntakeConstants {
        public static final int INTAKE_MOTOR_LEFT_ID = 1;
        public static final int INTAKE_MOTOR_RIGHT_ID = 2;
        public static final int EXTEND_MOTOR_ID = 3;
        public static final int BLOCKER_MOTOR_ID = 4;

        public static final double INTAKE_EXTEND = 1.0;
        public static final double INTAKE_RETRACT = 0.0;

        public static final double BLOCKER_EXTEND = 0.25;
        public static final double BLOCKER_RETRACT = 0;
    }

    public static class DyeConstants {
        public static final int DYE_ROTATE_MOTOR_ID = 61;
        public static final int DYE_WHEEL_MOTOR_ID = 62;
    }

    public static class TurretConstants {
        public static final int TOP_MOTOR_ID_LEFT = 51;
        public static final int TOP_MOTOR_ID_RIGHT = 52;
        public static final int HOOD_MOTOR_ID = 53;
        public static final int AZIMUTH_MOTOR_ID = 54;

        public static final double AZIMUTH_GEAR_RATIO = 53.2;
        public static final double AZIMUTH_LOWER_LIMIT = -1.0;
        public static final double AZIMUTH_UPPER_LIMIT = 1.0;
    }
}
