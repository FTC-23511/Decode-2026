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
    public static double VOLTAGE_SENSOR_POLLING_RATE = 10; // Hertz

    // Drive
    public static Pose2d END_POSE = new Pose2d();
    public static DistanceUnit DISTANCE_UNIT = DistanceUnit.INCH;
    public static AngleUnit ANGLE_UNIT = AngleUnit.RADIANS;

    public static double TRACK_WIDTH = 11.27362; // Inches
    public static double WHEEL_BASE = 11.50976; // Inches
    public static double MAX_VELOCITY = 7.75 * 12; // Inches/second
    public static double AUTO_MAX_VELOCITY = 50; // Inches/second
    public static double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / Math.hypot(TRACK_WIDTH / 2, WHEEL_BASE / 2);
    public static double AUTO_MAX_ANGULAR_VELOCITY = AUTO_MAX_VELOCITY / Math.hypot(TRACK_WIDTH / 2, WHEEL_BASE / 2);
    public static double PINPOINT_TELEOP_POLLING_RATE = 20; // Hertz

    public static PIDFCoefficients SWERVO_PIDF_COEFFICIENTS = new PIDFCoefficients(0.5, 0, 0.2, 0);
    public static double FR_ENCODER_OFFSET = 3.602; // Radians
    public static double FL_ENCODER_OFFSET = 3.753; // Radians
    public static double BL_ENCODER_OFFSET = 0.619; // Radians
    public static double BR_ENCODER_OFFSET = 2.149; // Radians

    public static PIDFCoefficients XY_COEFFICIENTS = new PIDFCoefficients(6, 0, 0.2, 0); // Coefficients for inches
    public static PIDFCoefficients HEADING_COEFFICIENTS = new PIDFCoefficients(4.2, 0, 0.23, 0); // Coefficients for radians
    public static double XY_TOLERANCE = 0.25; // Inches
    public static double HEADING_TOLERANCE = 0.05; // Radians
    public static double XY_MIN_OUTPUT = 8; // Inches/second
    public static double HEADING_MIN_OUTPUT = 0.15; // Radians/second

    // Intake
    public static double INTAKE_PIVOT_INTAKE = 0.62;
    public static double INTAKE_PIVOT_HIGH = 0.85;
    public static double INTAKE_PIVOT_HOLD = 0.58;
    public static double INTAKE_PIVOT_TRANSFER = 0.55;

    public static double INTAKE_FORWARD_SPEED = 0.7;
    public static double INTAKE_REVERSE_SPEED = -0.75;
    public static double INTAKE_TRANSFER_SPEED = 1.0; // TODO: needs to be tuned

    public static double MIN_DISTANCE_THRESHOLD = 0.00; // TODO: needs to be tuned
    public static double MAX_DISTANCE_THRESHOLD = 1.00; // TODO: needs to be tuned

    // Launcher
    public static double RAMP_ENGAGED = 0.3;
    public static double RAMP_DISENGAGED = 0.06;

    public static PIDFCoefficients FLYWHEEL_PIDF_COEFFICIENTS = new PIDFCoefficients(0.004, 0, 0, 0.00055); // Coefficients for ticks
    public static double M_S_TO_TICKS = 275; // Conversion factor for ball velocity in meters/second to launcher encoder ticks // TODO: needs to be tuned

    public static double LAUNCHER_FAR_VELOCITY = 7.8; // Meters/second // TODO: needs to be tuned
    public static double LAUNCHER_CLOSE_VELOCITY = 6.0; // Meters/second // TODO: needs to be tuned

    public static double MIN_HOOD_ANGLE = 16; // Degrees from horizontal // TODO: needs to be checked off CAD
    public static double MIN_HOOD_SERVO_POS = 0.07; // MUST MATCH WITH VALUE ABOVE // TODO: needs to be tuned
    public static double MAX_HOOD_ANGLE = 50; // Degrees from horizontal
    public static double MAX_HOOD_SERVO_POS = 1.0; // MUST MATCH WITH VALUE ABOVE // TODO: needs to be tuned

    public static double AUTONOMOUS_HOOD_ANGLE = 0.5; // Servo Position // TODO: needs to be tuned

    // Turret
    public static double TURRET_OFF_CENTER_FRONT_BACK = 0.0; // Inches // TODO: needs to be checked off CAD
    public static PIDFCoefficients TURRET_PIDF_COEFFICIENTS = new PIDFCoefficients(1.5, 0, 0.015, 0); // Coefficients for radians
    public static double TURRET_MIN_OUTPUT = 0.15; // Power
    public static double TURRET_POS_TOLERANCE = 0.03; // Radians
    public static double TURRET_ENCODER_OFFSET = 0.735; // Radians
    public static double MAX_TURRET_ANGLE = (110 / 360.0) * 2 * Math.PI; // Radians (only for one side of the turret, should be doubled for total range)
    public static Pose2d GOAL_POSE() { return new Pose2d((ALLIANCE_COLOR.equals(AllianceColor.BLUE) ? 72 : -72), 72, 0); } // Inches
}