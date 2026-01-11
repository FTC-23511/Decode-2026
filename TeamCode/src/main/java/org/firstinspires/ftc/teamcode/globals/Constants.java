package org.firstinspires.ftc.teamcode.globals;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;


import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Constants {
    // --- OP MODE STATE ---
    public enum OpModeType { AUTO, TELEOP }
    public static OpModeType OP_MODE_TYPE = OpModeType.TELEOP;
    public enum AllianceColor { BLUE, RED }
    public static AllianceColor ALLIANCE_COLOR = AllianceColor.BLUE;
    public static boolean TESTING_OP_MODE = false;

    // --- PINPOINT / ODOMETRY ---
    public static AngleUnit ANGLE_UNIT = AngleUnit.RADIANS;
    public static DistanceUnit DISTANCE_UNIT = DistanceUnit.INCH;
    public static Pose2d END_POSE = new Pose2d(0, 0, new Rotation2d(0));
    public static double PINPOINT_TELEOP_POLLING_RATE = 50.0;
    public static double PINPOINT_AUTO_POLLING_RATE = 100.0;

    // --- DRIVE TRAIN ---
    public static double TRACK_WIDTH = 11.0;
    public static double WHEEL_BASE = 13.5;
    public static double MAX_DRIVE_VELOCITY = 79.0;
    public static double MAX_ANGULAR_VELOCITY = 9.0;
    public static double JOYSTICK_DEAD_ZONE = 0.05;
    public static double FR_ENCODER_OFFSET = 0.8; // Radians
    public static double FL_ENCODER_OFFSET = 4.478; // Radians
    public static double BL_ENCODER_OFFSET = 6.106; // Radians
    public static double BR_ENCODER_OFFSET = 2.414; // Radians
    // --- DRIVE PID ---
    public static PIDFCoefficients SWERVO_PIDF_COEFFICIENTS = new PIDFCoefficients(0.5, 0, 0.01, 0);
    public static PIDFCoefficients XY_COEFFICIENTS = new PIDFCoefficients(4.0, 0, 0.2, 0);
    public static PIDFCoefficients HEADING_COEFFICIENTS = new PIDFCoefficients(5.0, 0, 0.3, 0);
    public static PIDFCoefficients TELEOP_HEADING_COEFFICIENTS = new PIDFCoefficients(3.0, 0, 0.1, 0);
    public static double XY_MIN_OUTPUT = 0.05, HEADING_MIN_OUTPUT = 0.05;
    public static double XY_TOLERANCE = 0.5, HEADING_TOLERANCE = 0.02;
    public static double STRAFING_SLEW_RATE_LIMIT = 4.0, TURNING_SLEW_RATE_LIMIT = 4.0;


    // --- LAUNCHER (FLY WHEEL & HOOD) ---
// Corrected to use PIDFController.PIDFCoefficients
    public static PIDFCoefficients FLYWHEEL_PIDF_COEFFICIENTS = new PIDFCoefficients(0.01, 0.00001, 0.0001, 0.0005);
// original is .005 f is 0.0005
    public static double FLYWHEEL_VEL_TOLERANCE = 50.0;
    public static double LAUNCHER_MAX_VELOCITY = 2500.0;
    public static double LAUNCHER_DEFAULT_ON_SPEED = 0.5; // Raw power if PID is off
    public static double MIN_HOOD_ANGLE = 10.0, MAX_HOOD_ANGLE = 45.0;
    public static double MIN_HOOD_SERVO_POS = 0.0, MAX_HOOD_SERVO_POS = 1.0;

    // --- INTAKE ---
    public static double INTAKE_FORWARD_SPEED = 1.0;
    public static double INTAKE_REVERSE_SPEED = -0.8;
    public static double INTAKE_TRANSFER_SPEED = 0.6;
}
