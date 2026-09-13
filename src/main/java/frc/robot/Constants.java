package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static class IntakeConstants {
        public static final int INTAKE_MOTOR_LEFT_ID = 1;
        public static final int INTAKE_MOTOR_RIGHT_ID = 2;
        public static final int EXTEND_MOTOR_ID = 3;
        public static final int BLOCKER_MOTOR_ID = 4;

        public static final double INTAKE_EXTEND = 1.0;
        public static final double INTAKE_RETRACT = 0.0;

        public static final double BLOCKER_EXTEND = -1.3;
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

        public static final double TURRET_SPEED = 8.0; // m/s

        public static final double HOOD_MIN_ANGLE = 10;
        public static final double HOOD_MAX_ANGLE = 45;
        public static final double HOOD_MIN_ROTATION = 0;
        public static final double HOOD_MAX_ROTATION = 3;
        public static final double HOOD_ANGLE_RANGE = HOOD_MAX_ANGLE-HOOD_MIN_ANGLE;
        public static final double HOOD_ROTATION_RANGE = HOOD_MAX_ROTATION-HOOD_MIN_ROTATION;

    }

    public static class UniverseConstants {
        public static double hubHeight = 1.82; // m

        public static double redHubX = Units.inchesToMeters(651.22 - 182.11);
        public static double redHubY = Units.inchesToMeters(158.84);
        public static double blueHubX = Units.inchesToMeters(182.11);
        public static double blueHubY = Units.inchesToMeters(158.84);

        public static Pose3d blueGoalPose = new Pose3d(blueHubX, blueHubY, hubHeight, new Rotation3d());

        public static Pose3d redGoalPose = new Pose3d(redHubX, redHubY, hubHeight, new Rotation3d());

        public static double fieldLength = Units.inchesToMeters(651.22);
        public static double fieldWidth = Units.inchesToMeters(317.69);
        public static double blueDepotFeedX = 2.5;
        public static double blueDepotFeedY = 6;

        public static double redOutpostFeedX = fieldLength - blueDepotFeedX;
        public static double redOutpostFeedY = blueDepotFeedY;

        public static double blueOutpostFeedX = blueDepotFeedX;
        public static double blueOutpostFeedY = fieldWidth - blueDepotFeedY;
        public static double redDepotFeedX = fieldLength - blueOutpostFeedX;
        public static double redDepotFeedY = blueOutpostFeedY;

        public static double fieldWidthMidpoint = fieldWidth / 2;
        public static double fieldLengthMidpoint = fieldLength / 2;

        public static double g = 9.81;
    }
}
