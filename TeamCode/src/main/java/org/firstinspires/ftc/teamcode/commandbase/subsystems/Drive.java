package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.swerve.coaxial.CoaxialSwerveDrivetrain;
import com.seattlesolvers.solverslib.gamepad.SlewRateLimiter;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.p2p.P2PController;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
public class Drive extends SubsystemBase {
    public final P2PController follower;
    public boolean headingLock = false;
    private final Robot robot = Robot.getInstance();
    public final CoaxialSwerveDrivetrain swerve;
    private final ElapsedTime timer;
    public static double ANGLE_OFFSET = 0.0;

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
                new PIDFController(XY_COEFFICIENTS).setMinOutput(XY_MIN_OUTPUT),
                (OP_MODE_TYPE.equals(OpModeType.TELEOP) ? new PIDFController(TELEOP_HEADING_COEFFICIENTS) : new PIDFController(HEADING_COEFFICIENTS)).setMinOutput(HEADING_MIN_OUTPUT),
                ANGLE_UNIT,
                XY_TOLERANCE,
                HEADING_TOLERANCE
        );
//        .setSlewRateLimiters(
//                new SlewRateLimiter(AUTO_STRAFING_SLEW_RATE_LIMIT),
//                new SlewRateLimiter(AUTO_STRAFING_SLEW_RATE_LIMIT),
//                new SlewRateLimiter(AUTO_TURNING_SLEW_RATE_LIMIT)
//        );

        timer = new ElapsedTime();

        if (OP_MODE_TYPE.equals(OpModeType.TELEOP)) {
            setPose(END_POSE);
        }
    }

    public void init() {
        follower.setTarget(END_POSE);
        if (OP_MODE_TYPE.equals(OpModeType.TELEOP) && !TESTING_OP_MODE) {
            setPose(END_POSE);
        }
        ANGLE_OFFSET = -0.085 * ALLIANCE_COLOR.getMultiplier();
    }

    public Pose2d getPose() {
        return new Pose2d(robot.pinpoint.getPosition(), DISTANCE_UNIT, ANGLE_UNIT).rotate(ANGLE_OFFSET);
    }

    public ChassisSpeeds getVelocity() {
        return new ChassisSpeeds(
                robot.pinpoint.getVelX(DistanceUnit.INCH),
                robot.pinpoint.getVelY(DistanceUnit.INCH),
                robot.pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
        );
    }

    public void setPose(Pose2d pose) {
        robot.pinpoint.setPosition(Pose2d.convertToPose2D(pose, DISTANCE_UNIT, ANGLE_UNIT));
    }

    public Pose2d turretPoseToDrivePose(Pose2d turretPose) {
        return new Pose2d(
                turretPose.getTranslation()
                        .plus(new Translation2d(-TURRET_OFF_CENTER_FRONT_BACK, 0)
                                .rotateBy(new Rotation2d(robot.turret.getPosition()))
                        ),
                turretPose.getRotation()
        );
    }

    public static boolean robotInZone(Pose2d robotPose) {
        robotZone.setPosition(robotPose.getX(), robotPose.getY());
        robotZone.setRotation(robotPose.getHeading());

        return robotZone.distanceTo(bigLaunchZone) <= Math.max(0, ZONE_TOLERANCE)
            || robotZone.distanceTo(smallLaunchZone) <= Math.max(0, ZONE_TOLERANCE);
    }

    @Override
    public void periodic() {
//        swerve.update(); // Not needed as we are using updateWithTargetVelocity() in the opModes
        robot.profiler.start("Drive Update");
        if (timer.milliseconds() > (1000 / (OP_MODE_TYPE.equals(OpModeType.AUTO) ? PINPOINT_AUTO_POLLING_RATE : PINPOINT_TELEOP_POLLING_RATE))) {
            robot.pinpoint.update();
            timer.reset();
        }
        robot.profiler.end("Drive Update");
    }

}
