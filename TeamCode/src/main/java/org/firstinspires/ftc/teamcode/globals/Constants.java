package org.firstinspires.ftc.teamcode.globals;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.controller.PIDFController;
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

        private int val;

        AllianceColor(int multiplier) {
            val = multiplier;
        }

        public int getMultiplier() {
            return val;
        }
    }

    public static OpModeType OP_MODE_TYPE;
    public static AllianceColor ALLIANCE_COLOR = AllianceColor.BLUE;
    public static double VOLTAGE_SENSOR_POLLING_RATE = 5; // Hertz
    public static double DEFAULT_VOLTAGE = 12.67; // Volts
    public static boolean PROBLEMATIC_TELEMETRY = false;
    public static boolean TESTING_OP_MODE = false;

    // Drive
    public static Pose2d END_POSE = new Pose2d();
    public static DistanceUnit DISTANCE_UNIT = DistanceUnit.INCH;
    public static AngleUnit ANGLE_UNIT = AngleUnit.RADIANS;

    public static double TRACK_WIDTH = 11; // Inches
    public static double WHEEL_BASE = 13.5; // Inches
    public static double MAX_DRIVE_VELOCITY = 7.75 * 12; // Inches/second
    public static double MAX_ANGULAR_VELOCITY = MAX_DRIVE_VELOCITY / Math.hypot(TRACK_WIDTH / 2, WHEEL_BASE / 2);
    public static double PINPOINT_TELEOP_POLLING_RATE = 10; // Hertz
    public static double PINPOINT_AUTO_POLLING_RATE = 100; // Hertz // TODO: needs to be tuned

    public static double STRAFING_SLEW_RATE_LIMIT = 6.7; // Joystick/second
    public static double TURNING_SLEW_RATE_LIMIT = 8.67; // Joystick/second
    public static double JOYSTICK_DEAD_ZONE = 0.02; // Joystick
    public static double MAX_TELEOP_HEADING_CORRECTION_VEL = 1.0; // Radians/second

    public static PIDFCoefficients SWERVO_PIDF_COEFFICIENTS = new PIDFCoefficients(0.5, 0, 0.2, 0);
    public static double FR_ENCODER_OFFSET = 0.8; // Radians
    public static double FL_ENCODER_OFFSET = 4.478; // Radians
    public static double BL_ENCODER_OFFSET = 6.106; // Radians
    public static double BR_ENCODER_OFFSET = 2.414; // Radians

    public static PIDFCoefficients XY_COEFFICIENTS = new PIDFCoefficients(4.5, 0, 0.4, 0); // Coefficients for inches
    public static PIDFCoefficients HEADING_COEFFICIENTS = new PIDFCoefficients(5, 0, 0.3, 0); // Coefficients for radians
    public static PIDFCoefficients TELEOP_HEADING_COEFFICIENTS = new PIDFCoefficients(6.7, 0, 0.25, 0); // Coefficients for radians
    public static double XY_TOLERANCE = 0.41; // Inches
    public static double HEADING_TOLERANCE = 0.041; // Radians
    public static double XY_MIN_OUTPUT = 12; // Inches/second
    public static double HEADING_MIN_OUTPUT = 2; // Radians/second

    // Intake

    public static double INTAKE_FORWARD_SPEED = 1.0;
    public static double INTAKE_REVERSE_SPEED = -1.0;
    public static double INTAKE_TRANSFER_SPEED = 1.0;

    public static double INTAKE_CURRENT_THRESHOLD = 3410; // Milliamps
    public static double INTAKE_UNJAM_TIME = 410.0; // Milliseconds


    // Launcher
    public static double RAMP_ENGAGED = 0.3;
    public static double RAMP_DISENGAGED = 0.06;

    public static PIDFCoefficients FLYWHEEL_PIDF_COEFFICIENTS = new PIDFCoefficients(0.01, 0, 0, 0.00052); // Coefficients for ticks
    public static double FLYWHEEL_VEL_TOLERANCE = 41; // Ticks
    public static double LAUNCHER_DEFAULT_ON_SPEED = 0.67; // Power

    public static final double GRAVITY = 9.81; // meters/second
    public static final double LAUNCHER_HEIGHT = 0.3302; // meters // 13 inches
    public static final double TARGET_HEIGHT = 1.0; // meters

    public static double LAUNCHER_VERY_FAR_VELOCITY = 7.0;
    public static double LAUNCHER_FAR_VELOCITY = 5.8; // Meters/second
    public static double LAUNCHER_CLOSE_VELOCITY = 4.51; // Meters/second
    public static double LAUNCHER_MAX_VELOCITY = 2500; // Ticks/second
    public static double LAUNCHER_MAX_BALL_VELOCITY = 100; // Meters/second // TODO: maybe actually deal with this later

    public static double MIN_HOOD_ANGLE = 10; // Degrees from horizontal
    public static double MIN_HOOD_SERVO_POS = 0; // MUST MATCH WITH VALUE ABOVE
    public static double MAX_HOOD_ANGLE = 45; // Degrees from horizontal
    public static double MAX_HOOD_SERVO_POS = 1 ; // Position // MUST MATCH WITH VALUE ABOVE
    public static double HOOD_COMPENSATION = 0.0067; // ticks to degrees

    public static Pose2d GOAL_POSE() { return new Pose2d(-72 * ALLIANCE_COLOR.getMultiplier(), 72, 0); } // Inches
    public static Pose2d APRILTAG_POSE() { return new Pose2d(-55.630 * ALLIANCE_COLOR.getMultiplier(), 58.346, 0); } // Feet
    public static double TURRET_BUFFER = (16.7 / 360.0) * 2 * Math.PI; // Radians // used for calculating drivetrain rotation in aimbot
}