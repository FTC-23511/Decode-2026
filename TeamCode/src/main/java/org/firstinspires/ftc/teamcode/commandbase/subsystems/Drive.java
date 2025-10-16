package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.SquIDFController;
import com.seattlesolvers.solverslib.drivebase.swerve.coaxial.CoaxialSwerveDrivetrain;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.p2p.P2PController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

public class Drive extends SubsystemBase {
    public final P2PController follower;
    private final Robot robot = Robot.getInstance();
    public final CoaxialSwerveDrivetrain swerve;
    private final ElapsedTime timer;
    private Pose2d lastPose;

    public Drive() {
        swerve = new CoaxialSwerveDrivetrain(
                TRACK_WIDTH,
                WHEEL_BASE,
                MAX_VELOCITY,
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
                new PIDFController(HEADING_COEFFICIENTS).setMinimumOutput(HEADING_MIN_OUTPUT),
                ANGLE_UNIT,
                XY_TOLERANCE,
                HEADING_TOLERANCE
        );

        timer = new ElapsedTime();
    }

    public Pose2d getPose() {
        if (OP_MODE_TYPE.equals(Constants.OpModeType.AUTO)
            || timer.milliseconds() > (1000 / Constants.PINPOINT_TELEOP_POLLING_RATE)
            || lastPose == null) {

            timer.reset();
            lastPose = new Pose2d(robot.pinpoint.getPosition(), DISTANCE_UNIT, ANGLE_UNIT);
        }

        return lastPose;
    }

    public void setPose(Pose2d pose) {
        robot.pinpoint.setPosition(Pose2d.convertToPose2D(pose, DISTANCE_UNIT, ANGLE_UNIT));
    }

    @Override
    public void periodic() {
//        swerve.update(); // Not needed as we are using updateWithTargetVelocity() in the opModes
        robot.pinpoint.update();
    }
}
