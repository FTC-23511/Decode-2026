package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.swerve.coaxial.CoaxialSwerveDrivetrain;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.SlewRateLimiter;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.p2p.P2PController;
import com.seattlesolvers.solverslib.util.MathUtils;
import com.seattlesolvers.solverslib.util.TelemetryEx;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;
import org.firstinspires.ftc.teamcode.gvf.Path;
import org.firstinspires.ftc.teamcode.gvf.PathData;
import org.firstinspires.ftc.teamcode.gvf.Spline;

@Config
public class Drive extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    public final CoaxialSwerveDrivetrain swerve;
    public final P2PController follower;

    public enum DriveState {
        FOLLOW_SPLINE,
        PID_TO_POINT,
        BRAKE,
        WAIT,
        DRIVE,
        IDLE
    }
    public DriveState driveState = DriveState.IDLE;

    private Pose2d robotPose = new Pose2d();
    private ChassisSpeeds robotVelocity = new ChassisSpeeds();
    private ChassisSpeeds targetVelocity = new ChassisSpeeds();

    private final ElapsedTime timer;
    public static double ANGLE_OFFSET = 0.0;

    private static final PolygonZone bigLaunchZone = new PolygonZone(new Point(72, 72), new Point(0, 0), new Point(-72, 72));
    private static final PolygonZone smallLaunchZone = new PolygonZone(new Point(-24, -72), new Point(0, -48), new Point(24, -72));
    private static final PolygonZone robotZone = new PolygonZone(14.5, 17.5);

    private Path path = null;
    public PathData pd;
    public static double centripetalScalar = 0.2;

    private Vector2d moveVector = new Vector2d(0, 0);
    private double turn = 0;

    private PIDController headingPID = new PIDController(0.55, 0, 0.002);

    private Pose2d targetPoint = new Pose2d(0, 0, 0);
    public static PIDController xPID = new PIDController(0.1, 0.0, 0.003);
    public static PIDController yPID = new PIDController(0.1, 0.0, 0.003);
    public static PIDController hPID = new PIDController(0.15, 0.0, 0.003);

    public static double xThresh = 1.0, yThresh = 1.0, hThresh = 2.0, waypointThresh = 3.0;
    private double xError = 0.0, yError = 0.0, hError = 0.0;
    private double maxPower = 1.0;
    private boolean isWaypoint = false;

    public Drive() {
        // Preserve Swerve Hardware Initialization
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

        timer = new ElapsedTime();
    }

    public void init() {
        follower.setTarget(END_POSE);
        if (OP_MODE_TYPE.equals(Constants.OpModeType.TELEOP) && !TESTING_OP_MODE) {
            setPose(END_POSE);
        }
        ANGLE_OFFSET = -0.085 * ALLIANCE_COLOR.getMultiplier();
    }

    public void setPose(Pose2d pose) {
        robot.pinpoint.setPosition(Pose2d.convertToPose2D(pose, DISTANCE_UNIT, ANGLE_UNIT));
    }

    public Pose2d getPose() {
        return robotPose;
    }

    public ChassisSpeeds getVelocity() {
        return robotVelocity;
    }

    public ChassisSpeeds getTargetVelocity() {
        return targetVelocity;
    }

    public void setTeleopVelocities(double forward, double strafe, double turn) {
        targetVelocity = new ChassisSpeeds(forward, strafe, turn);
    }

    public void drive(boolean fieldCentric) {
        driveState = DriveState.DRIVE;

        Vector2d moveVector = new Vector2d(targetVelocity.vxMetersPerSecond, targetVelocity.vyMetersPerSecond);
        if (fieldCentric) {
            moveVector = moveVector.rotateBy(Math.toDegrees(-robotPose.getHeading() + ANGLE_OFFSET));
        }

        if (moveVector.magnitude() <= 0.01 && Math.abs(targetVelocity.omegaRadiansPerSecond) <= 0.01) {
            stopAllMotors();
        } else {
            // Kinematic Bridge: Bridge X/Y/Turn to Swerve drive
            swerve.updateWithTargetVelocity(new ChassisSpeeds(moveVector.getX(), moveVector.getY(), targetVelocity.omegaRadiansPerSecond));
        }
    }

    public void update() {
        robot.profiler.start("Drive Localizer");

        if (timer.milliseconds() > (1000 / (OP_MODE_TYPE.equals(Constants.OpModeType.AUTO) ? PINPOINT_AUTO_POLLING_RATE : PINPOINT_TELEOP_POLLING_RATE))) {
            robot.pinpoint.update();
            robotPose = new Pose2d(robot.pinpoint.getPosition(), DISTANCE_UNIT, ANGLE_UNIT).rotate(ANGLE_OFFSET);
            robotVelocity = new ChassisSpeeds(robot.pinpoint.getVelX(DISTANCE_UNIT), robot.pinpoint.getVelY(DISTANCE_UNIT), robot.pinpoint.getHeadingVelocity(ANGLE_UNIT.getUnnormalized()));
            timer.reset();
        }

        robot.profiler.end("Drive Localizer");

        // Camera Hook
        if (robot.camera != null) {
            robot.camera.addPose(System.nanoTime() / 1e9, robotPose);
        }

        robot.profiler.start("Drive Update");

        // Control logic
        if(path != null) {
            driveState = DriveState.FOLLOW_SPLINE;
        }

        switch(driveState) {
            case FOLLOW_SPLINE:
                if (path == null) {
                    driveState = DriveState.IDLE;
                    break;
                }
                pd = path.update(robotPose);

                if(pd == null || path.isCompleted()) {
                    targetPoint = new Pose2d(path.getLastPose().getX(), path.getLastPose().getY(), path.getLastPose().getHeading());
                    isWaypoint = false;
                    path = null;
                    driveState = DriveState.PID_TO_POINT;
                    break;
                }

                Vector2d pathForward, pathCentripetal;
                pathForward = pd.vel;

                pathCentripetal = new Vector2d(0, pathForward.magnitude() * pathForward.magnitude() / pd.r * centripetalScalar);
                pathCentripetal = pathCentripetal.rotateBy(Math.toDegrees(Math.atan2(pathForward.getY(), pathForward.getX())));

                moveVector = pathForward.plus(pathCentripetal);
                double mag = moveVector.magnitude();
                moveVector = moveVector.rotateBy(Math.toDegrees(-robotPose.getHeading()));

                double pathRot = 0;
                if(Math.abs(pd.r) < Spline.MAX_RADIUS) {
                    pathRot = pathForward.magnitude() / mag * (TRACK_WIDTH / (2.0 * pd.r)) * (pd.reversed ? -1 : 1);
                }

                double targetHeading = pd.targetHeading;
                double headingError = MathUtils.normalizeRadians(targetHeading - robotPose.getHeading(), false);
                turn = pathRot + headingPID.calculate(MathUtils.clamp(headingError, -0.6, 0.6));

                double distRemaining = Math.hypot(robotPose.getX() - path.getLastPose().getX(), robotPose.getY() - path.getLastPose().getY());
                if (path.pathSegments.get(pd.index).decel && distRemaining <= 18) {
                    moveVector = moveVector.times(0.8 * Math.sqrt(distRemaining / 18) + 0.2);
                }

                // Kinematic Bridge for GVF
                swerve.updateWithTargetVelocity(new ChassisSpeeds(moveVector.getX(), moveVector.getY(), turn));
                break;
            case PID_TO_POINT:
                calculateErrors();
                PIDF();

                if (atPoint()) {
                    driveState = isWaypoint ? DriveState.WAIT : DriveState.BRAKE;
                }
                break;
            case BRAKE:
                stopAllMotors();
                driveState = DriveState.WAIT;
                break;
            case WAIT:
                if (!atPoint()) {
                    driveState = DriveState.PID_TO_POINT;
                }
                break;
            case DRIVE:
            case IDLE:
                break;
        }

        robot.profiler.end("Drive Update");
    }

    /**
     * Virtual Target Shift: Calculates the physical (X,Y) field coordinate the robot
     * is physically aiming at based on current manual ANGLE_OFFSET and DISTANCE_OFFSET.
     * Sets this as a permanent global target offset and resets manual offsets to zero.
     */
    public void applyVirtualTargetShift() {
        Pose2d robotPose = getPose(); // includes current ANGLE_OFFSET
        Pose2d baseGoal = GOAL_POSE(); // current adjusted goal

        Vector2d robotToGoal = new Vector2d(baseGoal).minus(new Vector2d(robotPose));
        double dist = robotToGoal.magnitude();
        double angle = robotToGoal.angle();

        // Polar-to-Cartesian kinematics to find physical aim point
        // angle_aim = perceived_angle - manual_offset
        // dist_aim = perceived_dist + manual_offset
        double angleAim = angle - ANGLE_OFFSET;
        double distAim = dist + Launcher.DISTANCE_OFFSET;

        double xAim = robotPose.getX() + distAim * Math.cos(angleAim);
        double yAim = robotPose.getY() + distAim * Math.sin(angleAim);

        // Store difference from the static base target (-72, 72)
        double trueGoalX = -72.0 * ALLIANCE_COLOR.getMultiplier();
        double trueGoalY = 72.0;

        TARGET_OFFSET_X = xAim - trueGoalX;
        TARGET_OFFSET_Y = yAim - trueGoalY;

        // Reset manual offsets to zero to prevent double-application
        ANGLE_OFFSET = 0;
        Launcher.DISTANCE_OFFSET = 0;
    }

    public void setPath(Path p) {
        this.path = p;
    }

    public void addPoint(Pose2d point, boolean reversed) {
        path.addPoint(point);
        path.setReversed(reversed);
    }

    private void calculateErrors(){
        double deltaX = (targetPoint.getX() - robotPose.getX());
        double deltaY = (targetPoint.getY() - robotPose.getY());

        // convert error into direction robot is facing
        xError = Math.cos(robotPose.getHeading()) * deltaX + Math.sin(robotPose.getHeading()) * deltaY;
        yError = -Math.sin(robotPose.getHeading()) * deltaX + Math.cos(robotPose.getHeading()) * deltaY;
        hError = MathUtils.normalizeRadians(targetPoint.getHeading() - robotPose.getHeading(), false);
    }

    private void PIDF(){
        double fwd = xPID.calculate(MathUtils.clamp(xError, -maxPower, maxPower));
        double strafe = yPID.calculate(MathUtils.clamp(yError, -maxPower, maxPower));
        double h = headingPID.calculate(MathUtils.clamp(hError, -maxPower, maxPower));

        // Kinematic Bridge for PID: Passing X/Y/Turn directly to Swerve
        swerve.updateWithTargetVelocity(new ChassisSpeeds(fwd, strafe, h));
    }

    private boolean atPoint() {
        if (isWaypoint) return Math.abs(xError) < waypointThresh && Math.abs(yError) < waypointThresh;
        return Math.abs(xError) < xThresh && Math.abs(yError) < yThresh && Math.abs(hError) < Math.toRadians(hThresh);
    }

    public void goToPoint(Pose2d targetPoint, double maxPower) {
        goToPoint(targetPoint, maxPower, false);
    };

    public void goToPoint(Pose2d targetPoint, double maxPower, boolean isWaypoint) {
        Pose2d lastTargetPoint = this.targetPoint;
        this.targetPoint = targetPoint;
        this.maxPower = maxPower;
        this.isWaypoint = isWaypoint;

        if (lastTargetPoint.getX() != targetPoint.getX() || lastTargetPoint.getY() != targetPoint.getY() || lastTargetPoint.getHeading() != targetPoint.getHeading()) {
            xPID.reset();
            yPID.reset();
            hPID.reset();
            driveState = DriveState.PID_TO_POINT;
        }
    }

    public void stopAllMotors() {
        swerve.updateWithTargetVelocity(new ChassisSpeeds(0, 0, 0));
    }

    public void drive(GamepadEx gamepadEx) {
        drive(gamepadEx, true);
    }

    public void drive(GamepadEx gamepadEx, boolean fieldCentric) {
        setTeleopVelocities(
                smoothControls(gamepadEx.getLeftY()),
                smoothControls(-1 * gamepadEx.getLeftX()),
                smoothControls(gamepadEx.getRightX())
        );

        drive(fieldCentric);
    }

    public double smoothControls(double value) {
        return 0.7 * Math.tan(0.96 * value);
    }

    public void updateDriveTelemetry(TelemetryEx telemetryEx) {
        telemetryEx.addData("Drivetrain : TargetPoint", "(" + targetPoint.getX() + ", " + targetPoint.getY() + ", " + targetPoint.getHeading() + ")");
        telemetryEx.addData("Drivetrain : PID xError", xError);
        telemetryEx.addData("Drivetrain : PID yError", yError);
        telemetryEx.addData("Drivetrain : PID hError", hError);

        if (path != null && pd != null) {
            telemetryEx.drawPath(path, pd);
            telemetryEx.drawRobot(robotPose);
            telemetryEx.drawPoints(path.repel);
        }
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

    public boolean robotInZone() {
        Pose2d robotPose = getPose();
        robotZone.setPosition(robotPose.getX(), robotPose.getY());
        robotZone.setRotation(robotPose.getHeading());

        return robotZone.distanceTo(bigLaunchZone) <= Math.max(0, ZONE_TOLERANCE)
                || robotZone.distanceTo(smallLaunchZone) <= Math.max(0, ZONE_TOLERANCE);
    }

    @Override
    public void periodic() {
        if (TESTING_OP_MODE) {
            return;
        }

        update();
    }
}
