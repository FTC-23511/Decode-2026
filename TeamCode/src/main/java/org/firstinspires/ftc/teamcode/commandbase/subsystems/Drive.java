package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;
import org.firstinspires.ftc.teamcode.globals.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.swerve.coaxial.CoaxialSwerveDrivetrain;
import com.seattlesolvers.solverslib.gamepad.SlewRateLimiter;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Twist2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.p2p.P2PController;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.globals.MathFunctions;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
public class Drive extends SubsystemBase {
    public final P2PController follower;
    public boolean headingLock = false;
    private final Robot robot = Robot.getInstance();
    public final CoaxialSwerveDrivetrain swerve;
    private final ElapsedTime timer;
    public static double ANGLE_OFFSET = 0.0;

    private Pose2d robotPose = new Pose2d();
    private ChassisSpeeds robotVelocity = new ChassisSpeeds();

    private static final PolygonZone bigLaunchZone = new PolygonZone(new Point(72, 72), new Point(0, 0), new Point(-72, 72));
    private static final PolygonZone smallLaunchZone = new PolygonZone(new Point(-24, -72), new Point(0, -48), new Point(24, -72));
    private static final PolygonZone robotZone = new PolygonZone(14.5, 17.5);

    public Drive() {
        swerve = new CoaxialSwerveDrivetrain(
                TRACK_WIDTH,
                WHEEL_BASE,
                MAX_DRIVE_VELOCITY,
                SWERVO_PIDF_COEFFICIENTS,
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
        ).setCachingTolerance(0.01, 0.01);

        follower = new P2PController(
                new PIDFController(XY_COEFFICIENTS).setMinOutput(XY_MIN_OUTPUT),
                (OP_MODE_TYPE.equals(OpModeType.TELEOP) ? new PIDFController(TELEOP_HEADING_COEFFICIENTS) : new PIDFController(AUTO_HEADING_COEFFICIENTS)).setMinOutput(HEADING_MIN_OUTPUT),
                ANGLE_UNIT,
                XY_TOLERANCE,
                HEADING_TOLERANCE
        ).setSlewRateLimiters(
                new SlewRateLimiter(AUTO_STRAFING_SLEW_RATE_LIMIT),
                new SlewRateLimiter(AUTO_TURNING_SLEW_RATE_LIMIT)
        );

        timer = new ElapsedTime();

        if (OP_MODE_TYPE.equals(OpModeType.TELEOP) && !TESTING_OP_MODE) {
            setPose(END_POSE);
            follower.setTarget(END_POSE);
        }
    }

    public void init() {
        if (OP_MODE_TYPE.equals(OpModeType.TELEOP) && !TESTING_OP_MODE) {
            setPose(END_POSE);
            follower.setTarget(END_POSE);
        }
    }

    public Pose2d getPose() {
        Pose2d currentPose = robotPose.rotate(ANGLE_OFFSET);

        if (OP_MODE_TYPE.equals(OpModeType.TELEOP)) {
            return currentPose;
        }

        double dt = timer.seconds();
        
        // Blend measured (robot-centric) and target (robot-centric) velocities
        ChassisSpeeds measuredVelRobot = getVelocity();
        ChassisSpeeds targetVelRobot = swerve.getTargetVelocity();

        double blendedVx = measuredVelRobot.vxMetersPerSecond + (targetVelRobot.vxMetersPerSecond - measuredVelRobot.vxMetersPerSecond) * DRIVE_VEL_PREDICT_ALPHA;
        double blendedVy = measuredVelRobot.vyMetersPerSecond + (targetVelRobot.vyMetersPerSecond - measuredVelRobot.vyMetersPerSecond) * DRIVE_VEL_PREDICT_ALPHA;
        double blendedOmega = measuredVelRobot.omegaRadiansPerSecond + (targetVelRobot.omegaRadiansPerSecond - measuredVelRobot.omegaRadiansPerSecond) * DRIVE_VEL_PREDICT_ALPHA;

        ChassisSpeeds blendedVelRobot = new ChassisSpeeds(blendedVx, blendedVy, blendedOmega);

        return currentPose.exp(
                new Twist2d(
                        blendedVelRobot.vxMetersPerSecond * dt * DRIVE_POS_PREDICT_INTEGRATION_SCALAR,
                        blendedVelRobot.vyMetersPerSecond * dt * DRIVE_POS_PREDICT_INTEGRATION_SCALAR,
                        blendedVelRobot.omegaRadiansPerSecond * dt * DRIVE_POS_PREDICT_INTEGRATION_SCALAR
                )
        );
    }

    public void applyVirtualTargetShift() {
        // 1. Get the current shot solution (which uses current ANGLE_OFFSET and DISTANCE_OFFSET)
        MathFunctions.VirtualGoalSolver.ShotSolution solution = robot.getShotSolution();

        // 2. Get the raw robot pose (no ANGLE_OFFSET)
        Pose2d rawPose = robotPose;

        // 3. Calculate physical turret position
        double robotHeadingDegrees = rawPose.getRotation().getDegrees();
        Vector2d turretOffsetField = Constants.TURRET_PHYSICAL_OFFSET.rotateBy(robotHeadingDegrees);
        Vector2d turretPosVec = new Vector2d(rawPose.getX(), rawPose.getY()).plus(turretOffsetField);

        // 4. Calculate physical landing point P
        // distance + manual distance offset (meters)
        double physicalDistMeters = solution.effectiveDistance + Launcher.DISTANCE_OFFSET;
        double physicalDistInches = physicalDistMeters / 0.0254;

        // physical turret heading (radians)
        double heading = solution.turretGlobalHeading.getRadians();

        double pX = turretPosVec.getX() + physicalDistInches * Math.cos(heading);
        double pY = turretPosVec.getY() + physicalDistInches * Math.sin(heading);

        // 5. Calculate Delta from base target
        // Base target is (-72 * multiplier, 72)
        double baseGoalX = -72 * Constants.ALLIANCE_COLOR.getMultiplier();
        double baseGoalY = 72;

        Constants.TARGET_OFFSET_X = pX - baseGoalX;
        Constants.TARGET_OFFSET_Y = pY - baseGoalY;

        // 6. Reset manual offsets
        ANGLE_OFFSET = 0;
        Launcher.DISTANCE_OFFSET = 0;
    }

    public ChassisSpeeds getVelocity() {
        return robotVelocity;
    }

    public void setPose(Pose2d pose) {
        // Map from FTC Cartesian (X right, Y forward) to OctoQuad (X forward, Y left)
        robot.octoQuad.setLocalizerPose(
                (int)(DistanceUnit.mmPerInch * pose.getY()),
                (int)(DistanceUnit.mmPerInch * -pose.getX()),
                (float) (pose.getHeading() - Math.PI / 2.0)
        );

        robotPose = pose;
    }

    /**
     * For turret (updates within ZONE_TOLERANCE)
     */
    public static boolean robotNearZone(Pose2d robotPose) {
        robotZone.setPosition(robotPose.getX(), robotPose.getY());
        robotZone.setRotation(robotPose.getHeading());

        return robotZone.distanceTo(bigLaunchZone) <= Math.max(0, ZONE_TOLERANCE)
            || robotZone.distanceTo(smallLaunchZone) <= Math.max(0, ZONE_TOLERANCE);
    }

    /**
     * For launcher (updates only in zone)
     */
    public static boolean robotInZone(Pose2d robotPose) {
        robotZone.setPosition(robotPose.getX(), robotPose.getY());
        robotZone.setRotation(robotPose.getHeading());

        return robotZone.isInside(bigLaunchZone) || robotZone.isInside(smallLaunchZone);
    }

    @Override
    public void periodic() {
        robot.profiler.start("Drive Update");
        if (timer.milliseconds() > (1000 / (OP_MODE_TYPE.equals(OpModeType.AUTO) ? PINPOINT_AUTO_POLLING_RATE : PINPOINT_TELEOP_POLLING_RATE))) {

            robot.octoQuad.readLocalizerData(robot.localizer);
            if (robot.localizer.crcOk) {
                // Map from OctoQuad (X forward, Y left) to FTC Cartesian (X right, Y forward)
                robotPose = new Pose2d(
                        -robot.localizer.posY_mm / DistanceUnit.mmPerInch,
                        robot.localizer.posX_mm / DistanceUnit.mmPerInch,
                        robot.localizer.heading_rad + Math.PI / 2.0
                );
                
                // OctoQuad velX_mmS is Forward tracking wheel velocity, velY_mmS is Strafe (Left) tracking wheel velocity
                // ChassisSpeeds natively uses vx as Forward and vy as Left.
                // This perfectly matches Pinpoint's original getVelX (Forward) and getVelY (Strafe).
                robotVelocity = new ChassisSpeeds(
                        robot.localizer.velX_mmS / DistanceUnit.mmPerInch,
                        robot.localizer.velY_mmS / DistanceUnit.mmPerInch,
                        robot.localizer.velHeading_radS
                );
            }

            timer.reset();

            if (robot.camera != null && robot.camera.enabled) {
                robot.camera.addPose(System.nanoTime() / 1e9, getPose());
            }
        }
        robot.profiler.end("Drive Update");
    }
}
