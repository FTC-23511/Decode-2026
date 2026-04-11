package org.firstinspires.ftc.teamcode.tuning.sensor;

import static org.firstinspires.ftc.teamcode.globals.Constants.ALLIANCE_COLOR;
import static org.firstinspires.ftc.teamcode.globals.Constants.GOAL_POSE;
import static org.firstinspires.ftc.teamcode.globals.Constants.TESTING_OP_MODE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.TelemetryEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.MathFunctions;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.Arrays;

@Config
@TeleOp(name = "CameraAngle", group = "Sensor")
public class CameraAngle extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;

    private final TelemetryEx telemetryEx = new TelemetryEx(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();
    private Pose2d lastKnownPose = new Pose2d();

    @Override
    public void initialize() {
        // Must have for all opModes
        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;
        TESTING_OP_MODE = true;

        // Resets the command scheduler
        super.reset();

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        // Driver controls
        // TODO: add controls here

        // Operator controls
        // TODO: add controls here
    }

    @Override
    public void run() {
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            robot.initHasMovement();
            timer = new ElapsedTime();
        }

        Pose2d robotPose = robot.camera.getCameraPose();

        if (robotPose != null) {
            lastKnownPose = robotPose;
        }

//        robot.camera.updateCameraResult(5);

        telemetryEx.addData("loop time", timer.milliseconds());
        timer.reset();

        if (lastKnownPose == null) {
            telemetryEx.addData("bot pose", null);
        } else {
            Pose2d cameraPose = robot.camera.getCameraPose();

            if (cameraPose != null) {
                double angle = robot.turret.angleToWall(cameraPose);
                telemetryEx.addData("Median Wall Angle", angle);
                try {
                    telemetryEx.addData("tY Offset", robot.camera.getTxOffset(lastKnownPose));
                    double offset = -robot.turret.angleToWall(cameraPose) * ALLIANCE_COLOR.getMultiplier();
                    double adjustment = robot.turret.goalAdjustmentLUT.get(offset);

                    Pose2d adjustedGoal;
                    if (adjustment < 0) {
                        adjustedGoal = new Pose2d(GOAL_POSE().getX() - (adjustment * ALLIANCE_COLOR.getMultiplier()), GOAL_POSE().getY(), GOAL_POSE().getHeading());
                    } else {
                        adjustedGoal = new Pose2d(GOAL_POSE().getX(), GOAL_POSE().getY() - adjustment, GOAL_POSE().getHeading());
                    }
                    telemetryEx.addData("offset", offset);
                    telemetryEx.addData("adjusted goal", adjustedGoal);
                    double globalHeadingTarget = Turret.posesToAngle(lastKnownPose, robot.turret.adjustedGoalPose());
                    telemetryEx.addData("globalHeadingTarget", globalHeadingTarget);
                    double[] errorsDriveTurret = Turret.angleToDriveTurretErrors(globalHeadingTarget);
                    telemetryEx.addData("errorsDriveTurret", Arrays.toString(errorsDriveTurret));
                } catch (Exception ignored) {
                    telemetryEx.addData("tY Offset", "out of bounds error");
                }
                telemetryEx.addData("turret position", robot.turret.getPosition());
                telemetryEx.addData("bot pose", lastKnownPose);
                double distance = Constants.GOAL_POSE().minus(lastKnownPose).getTranslation().getNorm() * DistanceUnit.mPerInch;
                telemetryEx.addData("Distance (m)", distance);
                telemetryEx.addData("Launcher Math Values", Arrays.toString(MathFunctions.distanceToLauncherValues(distance)));
            } else {
                telemetryEx.addData("Median Wall Angle", "medianWallAngle is Empty");
            }
        }

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        robot.updateLoop(telemetryEx);
    }

    @Override
    public void end() {
//        Constants.END_POSE = robot.drive.getPose();
    }
}