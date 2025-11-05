package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.swerve.coaxial.CoaxialSwerveDrivetrain;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.p2p.P2PController;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

public class Drive extends SubsystemBase {
    public final P2PController follower;
    public boolean headingLock = false;
    private final Robot robot = Robot.getInstance();
    public final CoaxialSwerveDrivetrain swerve;
    private final ElapsedTime timer;
    private Pose2d lastPose;

    public Drive() {
        swerve = new CoaxialSwerveDrivetrain(
                TRACK_WIDTH,
                WHEEL_BASE,
                OP_MODE_TYPE.equals(OpModeType.AUTO) ? AUTO_MAX_DRIVE_VELOCITY : MAX_DRIVE_VELOCITY,
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
                new PIDFController(XY_COEFFICIENTS).setMinimumOutput(XY_MIN_OUTPUT),
                new PIDFController(XY_COEFFICIENTS).setMinimumOutput(XY_MIN_OUTPUT),
                (OP_MODE_TYPE.equals(OpModeType.TELEOP) ? new PIDFController(TELEOP_HEADING_COEFFICIENTS) : new PIDFController(HEADING_COEFFICIENTS)).setMinimumOutput(HEADING_MIN_OUTPUT),
                ANGLE_UNIT,
                XY_TOLERANCE,
                HEADING_TOLERANCE
        );

        timer = new ElapsedTime();
    }

    public Pose2d getPose() {
        if (timer.milliseconds() > (1000 / (OP_MODE_TYPE.equals(OpModeType.AUTO) ? PINPOINT_AUTO_POLLING_RATE : Constants.PINPOINT_TELEOP_POLLING_RATE))
            || lastPose == null) {

            timer.reset();
            lastPose = new Pose2d(robot.pinpoint.getPosition(), DISTANCE_UNIT, ANGLE_UNIT);
        }

        return new Pose2d(
                lastPose.getX(),
                lastPose.getY(),
                MathUtils.normalizeRadians(lastPose.getHeading(), false)
        );
    }

    public void setPose(Pose2d pose) {
        robot.pinpoint.setPosition(Pose2d.convertToPose2D(pose, DISTANCE_UNIT, ANGLE_UNIT));
    }

    public void setMaxSpeed(double maxSpeed) {
        swerve.setMaxSpeed(maxSpeed);
    }

    @Override
    public void periodic() {
//        swerve.update(); // Not needed as we are using updateWithTargetVelocity() in the opModes
        robot.pinpoint.update();
    }

    public void init() {
        if (OP_MODE_TYPE.equals(OpModeType.TELEOP) && END_POSE != null) {
            setPose(END_POSE);
        } else {
            follower.setTarget(new Pose2d());
        }
    }
}
