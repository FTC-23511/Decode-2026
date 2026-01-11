package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.swerve.coaxial.CoaxialSwerveDrivetrain;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.p2p.P2PController;

import org.firstinspires.ftc.teamcode.globals.Robot;

/**
 * Drive Subsystem
 * Manages the 4-module Coaxial Swerve Drivetrain and field-relative positioning.
 * Utilizes a P2P (Point-to-Point) controller for autonomous movement and heading stabilization.
 */
@Config
public class Drive extends SubsystemBase {
    // Controller for automated movement and maintaining heading lock
    public final P2PController follower;

    // State flags for driver assistance
    public boolean headingLock = false;
    public static boolean unsureXY = false;

    private final Robot robot = Robot.getInstance();
    public final CoaxialSwerveDrivetrain swerve;
    private final ElapsedTime timer;

    public Drive() {
        // --- Swerve Drivetrain Initialization ---
        // Configures the kinematics (Track Width/Wheel Base) and assigns the hardware
        swerve = new CoaxialSwerveDrivetrain(
                TRACK_WIDTH,
                WHEEL_BASE,
                MAX_DRIVE_VELOCITY,
                SWERVO_PIDF_COEFFICIENTS, // PID for individual module rotation
                new MotorEx[]{
                        robot.FRmotor,
                        robot.FLmotor,
                        robot.BLmotor,
                        robot.BRmotor
                },
                new CRServoEx[]{
                        robot.FRswervo,
                        robot.FLswervo,
                        robot.BLswervo,
                        robot.BRswervo
                }
        ).setCachingTolerance(0.01, 0.01); // Optimized to ignore tiny fluctuations in encoder data

        // --- P2P Follower Configuration ---
        // Sets up three PID loops: X position, Y position, and Heading (Angle)
        follower = new P2PController(
                new PIDFController(XY_COEFFICIENTS).setMinOutput(XY_MIN_OUTPUT),
                new PIDFController(XY_COEFFICIENTS).setMinOutput(XY_MIN_OUTPUT),
                // Uses more aggressive heading coefficients in Auto vs TeleOp
                (OP_MODE_TYPE.equals(OpModeType.TELEOP) ?
                        new PIDFController(TELEOP_HEADING_COEFFICIENTS) :
                        new PIDFController(HEADING_COEFFICIENTS)).setMinOutput(HEADING_MIN_OUTPUT),
                ANGLE_UNIT,
                XY_TOLERANCE,
                HEADING_TOLERANCE
        );

        timer = new ElapsedTime();

        // Alliance Handoff: Carry over the end position from Autonomous to TeleOp
        if (OP_MODE_TYPE.equals(OpModeType.TELEOP)) {
            setPose(END_POSE);
        }
    }

    /**
     * Retrieves the current field-relative position from the Pinpoint hardware.
     */
    public Pose2d getPose() {
        return new Pose2d(robot.pinpoint.getPosition(), DISTANCE_UNIT, ANGLE_UNIT);
    }

    /**
     * Manually updates the robot's coordinates in the odometry computer.
     */
    public void setPose(Pose2d pose) {
        robot.pinpoint.setPosition(Pose2d.convertToPose2D(pose, DISTANCE_UNIT, ANGLE_UNIT));
    }

    @Override
    public void periodic() {
        // Polls the Pinpoint sensor at the specified rate (higher in Auto for accuracy)
        if (timer.milliseconds() > (1000 / (OP_MODE_TYPE.equals(OpModeType.AUTO) ?
                PINPOINT_AUTO_POLLING_RATE : PINPOINT_TELEOP_POLLING_RATE))) {
            robot.pinpoint.update();
            timer.reset();
        }
    }

    /**
     * Initialization logic to sync the PID target with physical reality.
     * Prevents the robot from aggressively "snapping" to a 0-heading when first enabled.
     */
    public void init() {
        // Record where we actually are at match start
        Pose2d currentPose = getPose();

        // Initialize the PID target to current position to prevent startup jerking
        follower.setTarget(currentPose);

        if (OP_MODE_TYPE.equals(OpModeType.TELEOP) && !TESTING_OP_MODE) {
            // Apply the static END_POSE from Autonomous
            setPose(END_POSE);
            unsureXY = true; // Indicates the robot hasn't localized against a field wall yet
        }
    }
}
