package org.firstinspires.ftc.teamcode.globals;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Constants {
    // General
    public enum OpModeType {
        AUTO,
        TELEOP
    }

    public enum AllianceColor {
        BLUE(1), RED(-1);

        private final int multiplier;

        AllianceColor(int multiplier) {
            this.multiplier = multiplier;
        }

        public int getMultiplier() {
            return multiplier;
        }
    }

    public static OpModeType OP_MODE_TYPE;
    public static AllianceColor ALLIANCE_COLOR = AllianceColor.BLUE;
    public static double VOLTAGE_SENSOR_POLLING_RATE = 5; // Hertz
    public static double DEFAULT_VOLTAGE = 12.67; // Volts
    public static boolean PROBLEMATIC_TELEMETRY = false;
    public static boolean ALL_TELEMETRY = true;
    public static boolean TESTING_OP_MODE = false;

    // Drive
    public static Pose2d END_POSE = new Pose2d();
    public static DistanceUnit DISTANCE_UNIT = DistanceUnit.INCH;
    public static AngleUnit ANGLE_UNIT = AngleUnit.RADIANS;

    public static double TRACK_WIDTH = 11.27362; // Inches
    public static double WHEEL_BASE = 11.50976; // Inches
    public static double MAX_DRIVE_VELOCITY = 7.75 * 12; // Inches/second
    public static double MAX_ANGULAR_VELOCITY = MAX_DRIVE_VELOCITY / Math.hypot(TRACK_WIDTH / 2, WHEEL_BASE / 2);
    public static double PINPOINT_TELEOP_POLLING_RATE = 10; // Hertz
    public static double PINPOINT_AUTO_POLLING_RATE = 100; // Hertz // TODO: needs to be tuned

    public static double AUTO_STRAFING_SLEW_RATE_LIMIT = 250; // inches/second
    public static double AUTO_TURNING_SLEW_RATE_LIMIT = 1000; // radians/second // TODO: TUNE

    public static double STRAFING_SLEW_RATE_LIMIT = 6.7; // Joystick/second
    public static double TURNING_SLEW_RATE_LIMIT = 8.67; // Joystick/second
    public static double JOYSTICK_DEAD_ZONE = 0.02; // Joystick
    public static double MAX_TELEOP_HEADING_CORRECTION_VEL = 1.0; // Radians/second

    public static PIDFCoefficients SWERVO_PIDF_COEFFICIENTS = new PIDFCoefficients(0.5, 0, 0.2, 0);
    public static double FR_ENCODER_OFFSET = 3.602; // Radians
    public static double FL_ENCODER_OFFSET = 3.753; // Radians
    public static double BL_ENCODER_OFFSET = 2.0; // Radians
    public static double BR_ENCODER_OFFSET = 1.12; // Radians

    public static PIDFCoefficients XY_COEFFICIENTS = new PIDFCoefficients(4.5, 0, 0.4, 0); // Coefficients for inches
    public static PIDFCoefficients HEADING_COEFFICIENTS = new PIDFCoefficients(5, 0, 0.3, 0); // Coefficients for radians
    public static PIDFCoefficients TELEOP_HEADING_COEFFICIENTS = new PIDFCoefficients(6.7, 0, 0.25, 0); // Coefficients for radians
    public static PIDFCoefficients AIMBOT_COEFFICIENTS = new PIDFCoefficients(3.5, 0, 0, 0); // Coefficients for radians
    public static double XY_TOLERANCE = 0.41; // Inches
    public static double HEADING_TOLERANCE = 0.041; // Radians
    public static double XY_MIN_OUTPUT = 12; // Inches/second
    public static double HEADING_MIN_OUTPUT = 2; // Radians/second

    public static boolean ENABLE_ZONE_CONTROL = true;
    public static double ZONE_TOLERANCE = 10.0;

    // Intake
    public static double INTAKE_FORWARD_SPEED = 1.0;
    public static double INTAKE_REVERSE_SPEED = -1.0;
    public static double INTAKE_TRANSFER_SPEED = 1.0;

    public static double INTAKE_CURRENT_THRESHOLD = 3410; // Milliamps
    public static double INTAKE_UNJAM_TIME = 410.0; // Milliseconds

    public static double INTAKE_DISTANCE_THRESHOLD = 6.867; // cm
    public static double INTAKE_DISTANCE_TIME = 441; // milliseconds
//    public static double FRONT_DISTANCE_THRESHOLD = 0.00; // TODO: needs to be tuned
//    public static double BACK_DISTANCE_THRESHOLD = 0.00; // TODO: needs to be tuned

    // Launcher
    public static double RAMP_ENGAGED = 0.52;
    public static double RAMP_DISENGAGED = 0.26;

    public static PIDFCoefficients FLYWHEEL_PIDF_COEFFICIENTS = new PIDFCoefficients(0.01, 0, 0, 0.00045); // Coefficients for ticks
    public static double FLYWHEEL_VEL_TOLERANCE = 41; // Ticks
    public static double LAUNCHER_DEFAULT_ON_SPEED = 0.67; // Power

    public static final double GRAVITY = 9.81; // meters/second
    public static final double LAUNCHER_HEIGHT = 0.3302; // meters // 13 inches
    public static final double TARGET_HEIGHT = 1.0; // meters

    public static double LAUNCHER_VERY_FAR_VELOCITY = 7.0;
    public static double LAUNCHER_FAR_VELOCITY = 5.8; // Meters/second
    public static double LAUNCHER_CLOSE_VELOCITY = 4.51; // Meters/second
    public static double LAUNCHER_MAX_VELOCITY = 2500; // Ticks/second
    public static double LAUNCHER_MAX_BALL_VELOCITY = 12; // Meters/second // TODO: tune this to potentially be lower

    public static double MIN_HOOD_ANGLE = 20; // Degrees from horizontal
    public static double MIN_HOOD_SERVO_POS = 0.18; // MUST MATCH WITH VALUE ABOVE
    public static double MAX_HOOD_ANGLE = 45; // Degrees from horizontal
    public static double MAX_HOOD_SERVO_POS = 0.82; // Position // MUST MATCH WITH VALUE ABOVE

    // Turret
    public static PIDFCoefficients TURRET_LARGE_PIDF_COEFFICIENTS = new PIDFCoefficients(0.467, 0.0, 0.028, 0.0); // Coefficients for radians
    public static PIDFCoefficients TURRET_SMALL_PIDF_COEFFICIENTS = new PIDFCoefficients(0.467, 0.0, 0.028, 0.0); // Coefficients for radians
    public static double TURRET_VEL_FF = 0.0267; // Coefficient for radians/second
    public static double TURRET_OPEN_F = 0.029; // Power
    public static double TURRET_POS_TOLERANCE = 0.0167; // Radians
    public static double TURRET_TOLERANCE_SCALING(double distance) {
        return 1.0 / (10 * (distance + 1));
    }
    public static double TURRET_VEL_TOLERANCE = Double.POSITIVE_INFINITY; // Radians/second
    public static double TURRET_VEL_FILTER = 20; // Radians/second
    public static double TURRET_LAST_VEL_ENTRIES = 5;
    public static double TURRET_THRESHOLD = 0.15; // Radians
    public static double TURRET_MIN_OUTPUT = 0.00; // Power
    public static double TURRET_SMALL_MAX_OUTPUT = 0.067; // Power
    public static double TURRET_LARGE_MAX_OUTPUT = 1.0; // Power
    public static double TURRET_ENCODER_OFFSET = 3.60; // Radians
    // Quadrature Encoder CPR = 8192, and gear ratio is 180:32 (32t is encoder)
//    public static double TURRET_RADIANS_PER_TICK = (2.0 * Math.PI) / (8192 * (180.0 / 32)); // Radians
    public static double TURRET_RADIANS_PER_TICK = (-1.5636687150596416 - 1.576019942982521) / (-7572 - 7753); // Radians/tick
    // (-1.5636687150596416 - 1.576019942982521) / (-7572 - 7753)

//    public static PIDFController.IntegrationBehavior TURRET_INTEGRATION_BEHAVIOR = PIDFController.IntegrationBehavior.CLEAR_AT_SP;
//    public static double TURRET_MIN_INTEGRAL = -0.2;
//    public static double TURRET_MAX_INTEGRAL = 0.2;
//    public static double TURRET_INTEGRATION_DECAY = 0.7;

    public static double TURRET_OFF_CENTER_FRONT_BACK = 2.186; // Inches
    private final double TURRET_LIMELIGHT_OFFSET = -4.124; // inches // 0.10475 m // USED ONLY FOR LIMELIGHT PIPELINE
    public static double MAX_TURRET_ANGLE = 2.65; // Radians (only for one side of the turret)
    public static double MIN_TURRET_ANGLE = -2.55; // Radians (only for one side of the turret)
    public static boolean TURRET_SYNCED = false;
    public static double TURRET_SYNC_OFFSET = 0.0;

    public static float CAMERA_CLOSE_DECIMATION = 3;
    public static float CAMERA_FAR_DECIMATION = 2;
    public static boolean USE_CLOSE_DECIMATION = true;
    public static double TEST_DISTANCE = 80; // Inches
    public static double DECIMATION_THRESHOLD = 80; // Inches

    public static Pose2d GOAL_POSE() {
        return new Pose2d(-72 * ALLIANCE_COLOR.getMultiplier(), 72, 0); // Inches
    }

    public static double GOAL_LIP = 0.45; // Meters
    public static double BACKBOARD_Y_OFFSET = 0.1; // Meters
    public static double LIP_BUFFER = 8 * DistanceUnit.mPerInch; // Meters

    public static Pose2d APRILTAG_POSE() {
        return new Pose2d(-55.630 * ALLIANCE_COLOR.getMultiplier(), 58.346, 0); // Inches
    }
}