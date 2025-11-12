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
        RED,
        BLUE
    }

    public static OpModeType OP_MODE_TYPE;
    public static AllianceColor ALLIANCE_COLOR = AllianceColor.BLUE;
    public static double VOLTAGE_SENSOR_POLLING_RATE = 5; // Hertz

    // Drive
    public static Pose2d END_POSE = new Pose2d();
    public static DistanceUnit DISTANCE_UNIT = DistanceUnit.INCH;
    public static AngleUnit ANGLE_UNIT = AngleUnit.RADIANS;

    public static double TRACK_WIDTH = 11.27362; // Inches
    public static double WHEEL_BASE = 11.50976; // Inches
    public static double MAX_DRIVE_VELOCITY = 7.75 * 12; // Inches/second
    public static double AUTO_MAX_DRIVE_VELOCITY = 50; // Inches/second
    public static double MAX_ANGULAR_VELOCITY = MAX_DRIVE_VELOCITY / Math.hypot(TRACK_WIDTH / 2, WHEEL_BASE / 2);
    public static double AUTO_MAX_ANGULAR_VELOCITY = AUTO_MAX_DRIVE_VELOCITY / Math.hypot(TRACK_WIDTH / 2, WHEEL_BASE / 2);
    public static double PINPOINT_TELEOP_POLLING_RATE = 10; // Hertz
    public static double PINPOINT_AUTO_POLLING_RATE = 150; // Hertz // TODO: needs to be tuned

    public static double STRAFING_SLEW_RATE_LIMIT = 6.7; // Joystick/second
    public static double TURNING_SLEW_RATE_LIMIT = 6.7; // Joystick/second
    public static double JOYSTICK_DEAD_ZONE = 0.02; // Joystick
    public static double MAX_TELEOP_HEADING_CORRECTION_VEL = 1.0; // Radians/second

    public static PIDFCoefficients SWERVO_PIDF_COEFFICIENTS = new PIDFCoefficients(0.5, 0, 0.2, 0);
    public static double FR_ENCODER_OFFSET = 3.602; // Radians
    public static double FL_ENCODER_OFFSET = 3.753; // Radians
    public static double BL_ENCODER_OFFSET = 0.65; // Radians
    public static double BR_ENCODER_OFFSET = 1.89; // Radians

    public static PIDFCoefficients XY_COEFFICIENTS = new PIDFCoefficients(8, 0, 0.25, 0); // Coefficients for inches
    public static PIDFCoefficients HEADING_COEFFICIENTS = new PIDFCoefficients(10, 0, 0.67, 0); // Coefficients for radians
    public static PIDFCoefficients TELEOP_HEADING_COEFFICIENTS = new PIDFCoefficients(6.7, 0, 0.25, 0); // Coefficients for radians
    public static double XY_TOLERANCE = 0.41; // Inches
    public static double HEADING_TOLERANCE = 0.041; // Radians
    public static double XY_MIN_OUTPUT = 12; // Inches/second
    public static double HEADING_MIN_OUTPUT = 0.5; // Radians/second

    // Intake
    public static double INTAKE_PIVOT_FORWARD = 0.5841;
    public static double INTAKE_PIVOT_HOLD = 0.5;
    public static double INTAKE_PIVOT_TRANSFER = INTAKE_PIVOT_FORWARD;

    public static double INTAKE_FORWARD_SPEED = 1.0;
    public static double INTAKE_REVERSE_SPEED = -1.0;
    public static double INTAKE_TRANSFER_SPEED = 1.0;

    public static double INTAKE_CURRENT_THRESHOLD = 3410; // Milliamps
    public static double INTAKE_UNJAM_TIME = 410.0; // Milliseconds

    public static double MIN_DISTANCE_THRESHOLD = 0.00; // TODO: needs to be tuned
    public static double MAX_DISTANCE_THRESHOLD = 1.00; // TODO: needs to be tuned

    // Launcher
    public static double RAMP_ENGAGED = 0.3;
    public static double RAMP_DISENGAGED = 0.06;

    public static PIDFCoefficients FLYWHEEL_PIDF_COEFFICIENTS = new PIDFCoefficients(0.004, 0, 0, 0.00055); // Coefficients for ticks
    public static double FLYWHEEL_VEL_TOLERANCE = 41; // Ticks
    public static double LAUNCHER_DEFAULT_ON_SPEED = 0.67; // Power

    public static final double GRAVITY = 9.81; // meters/second
    public static final double LAUNCHER_HEIGHT = 0.3302; // meters // 13 inches
    public static final double TARGET_HEIGHT = 1.0; // meters

    public static double LAUNCHER_FAR_VELOCITY = 6.0; // Meters/second
    public static double LAUNCHER_CLOSE_VELOCITY = 4.51; // Meters/second
    public static double LAUNCHER_MAX_VELOCITY = 2000; // Ticks/second

    public static double MIN_HOOD_ANGLE = 20; // Degrees from horizontal // TODO: needs to be checked off CAD
    public static double MIN_HOOD_SERVO_POS = 0.00; // MUST MATCH WITH VALUE ABOVE
    public static double MAX_HOOD_ANGLE = 50; // Degrees from horizontal // TODO: needs to be checked off CAD
    public static double MAX_HOOD_SERVO_POS = 1.0; // Position // MUST MATCH WITH VALUE ABOVE
    public static double MIN_LL_HOOD_ANGLE = 16; // Degrees from horizontal // Minimum hood angle for Limelight to be able to see AprilTags from anywhere on the field// TODO: needs to be tuned

    // Turret
    public static double TURRET_OFF_CENTER_FRONT_BACK = 0.0; // Inches // TODO: needs to be checked off CAD
    public static PIDFCoefficients TURRET_PIDF_COEFFICIENTS = new PIDFCoefficients(1.4, 0, 0.023, 0); // Coefficients for radians
    public static PIDFCoefficients LIMELIGHT_LARGE_PIDF_COEFFICIENTS = new PIDFCoefficients(0.02, 0, 0, 0); // Coefficients for radians
    public static PIDFCoefficients LIMELIGHT_SMALL_PIDF_COEFFICIENTS = new PIDFCoefficients(0.06, 0, 0.0023, 0); // Coefficients for radians
    public static double LIMELIGHT_PID_THRESHOLD = 4; // LL TY Degrees
    public static double TURRET_TY_TOLERANCE = 1.967; // LL TY Degrees
    public static double TURRET_POS_TOLERANCE = 0.067; // Radians
    public static double TURRET_MIN_OUTPUT = 0.15; // Power
    public static double TURRET_ENCODER_OFFSET = 0.735; // Radians
    public static double MAX_TURRET_ANGLE = (115 / 360.0) * 2 * Math.PI; // Radians (only for one side of the turret, should be doubled for total range)
    public static Pose2d GOAL_POSE() { return new Pose2d((ALLIANCE_COLOR.equals(AllianceColor.BLUE) ? -72 : 72), 72, 0); } // Inches
    public static double TURRET_BUFFER = (16.7 / 360.0) * 2 * Math.PI; // Radians // used for calculating drivetrain rotation in aimbot
}