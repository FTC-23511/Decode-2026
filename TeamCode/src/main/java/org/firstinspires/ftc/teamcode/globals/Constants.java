package org.firstinspires.ftc.teamcode.globals;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.geometry.Pose2d;

@Config
public class Constants {
    public enum OpModeType {
        AUTO,
        TELEOP
    }
    public enum AllianceColor {
        RED,
        BLUE
    }

    // Drive
    public static OpModeType OP_MODE_TYPE;
    public static AllianceColor ALLIANCE_COLOR = AllianceColor.BLUE;
    public static Pose2d END_POSE = new Pose2d();

    public static double TRACK_WIDTH = 11.27362; // Inches
    public static double WHEEL_BASE = 11.50976; // Inches
    public static double MAX_VELOCITY = 7.75 * 12; // Inches/second
    public static double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / Math.hypot(TRACK_WIDTH/2, WHEEL_BASE/2);
    public static double PINPOINT_TELEOP_POLLING_RATE = 20; // Hertz
    public static PIDFCoefficients SWERVO_PIDF_COEFFICIENTS = new PIDFCoefficients(0.6, 0, 0.2, 0);
    public static double FR_ENCODER_OFFSET = 3.602; // Radians
    public static double FL_ENCODER_OFFSET = 3.753; // Radians
    public static double BL_ENCODER_OFFSET = 0.619; // Radians
    public static double BR_ENCODER_OFFSET = 2.149; // Radians
    public static PIDFCoefficients XY_COEFFICIENTS = new PIDFCoefficients(0, 0, 0, 0); // Coefficients for inches
    public static PIDFCoefficients HEADING_COEFFICIENTS = new PIDFCoefficients(0, 0, 0, 0); // Coefficients for radians
    public static double XY_TOLERANCE = 0.25; // Inches
    public static double HEADING_TOLERANCE = 0.1; // Radians

    // Intake
    public static double INTAKE_PIVOT_TRANSFER_POS = 0.00;
    public static double INTAKE_PIVOT_INTAKE_POS = 0.00;
    public static double INTAKE_PIVOT_READY_INTAKE_POS = 0.00;

    public static double INTAKE_FORWARD_SPEED = 0.00;
    public static double INTAKE_REVERSE_SPEED = 0.00;
    public static double INTAKE_HOLD_SPEED = 0.00;

    public static double MIN_DISTANCE_THRESHOLD = 0.00;
    public static double MAX_DISTANCE_THRESHOLD = 0.00;

    public static double TURRET_SET_POSITION_TUNER = 0.00;

    // Launcher
}
