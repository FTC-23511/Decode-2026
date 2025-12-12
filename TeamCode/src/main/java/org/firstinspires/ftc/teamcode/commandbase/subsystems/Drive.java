package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

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

public class Drive extends SubsystemBase {
    public final P2PController follower;
    public boolean headingLock = false;
    public boolean unsureXY = false;
    private final Robot robot = Robot.getInstance();
    public final CoaxialSwerveDrivetrain swerve;
    private final ElapsedTime timer;

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

        timer = new ElapsedTime();

        if (OP_MODE_TYPE.equals(OpModeType.TELEOP)) {
            setPose(END_POSE);
        }
    }

    public Pose2d getPose() {
        return new Pose2d(robot.pinpoint.getPosition(), DISTANCE_UNIT, ANGLE_UNIT);
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

    @Override
    public void periodic() {
//        swerve.update(); // Not needed as we are using updateWithTargetVelocity() in the opModes
        if (timer.milliseconds() > (1000 / (OP_MODE_TYPE.equals(OpModeType.AUTO) ? PINPOINT_AUTO_POLLING_RATE : PINPOINT_TELEOP_POLLING_RATE))) {
            robot.pinpoint.update();
        }
    }

    public void init() {
        follower.setTarget(new Pose2d());
    }
}
