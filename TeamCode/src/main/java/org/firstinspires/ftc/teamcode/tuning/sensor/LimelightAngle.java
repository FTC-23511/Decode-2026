package org.firstinspires.ftc.teamcode.tuning.sensor;

import static org.firstinspires.ftc.teamcode.globals.Constants.ALLIANCE_COLOR;
import static org.firstinspires.ftc.teamcode.globals.Constants.GOAL_POSE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp(name = "LimelightAngle", group = "Sensor")
public class LimelightAngle extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;

    private final TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();
    private Pose2d lastKnownPose = new Pose2d();

    @Override
    public void initialize() {
        // Must have for all opModes
        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;

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

        Pose2d robotPose = robot.turret.getLimelightPose();

        if (robotPose != null) {
            lastKnownPose = robotPose;
        }

        robot.turret.updateLLResult(5);

        telemetryData.addData("loop time", timer.milliseconds());
        timer.reset();

        if (lastKnownPose == null) {
            telemetryData.addData("bot pose", null);
        } else {
            Pose2d llPose = robot.turret.getLimelightPose();

            if (llPose != null) {
                robot.turret.updateMedianReadings(llPose);
            }

            if (robot.turret.medianWallAngle.size() > 10) {
                robot.turret.medianWallAngle.remove(0);
            }

            if (!robot.turret.medianWallAngle.isEmpty()) {
                double angle = robot.turret.getMedianWallAngle();
                telemetryData.addData("Median Y", angle);
                try {
                    telemetryData.addData("tY Offset", robot.turret.getTyOffset(lastKnownPose));
                    double offset = -robot.turret.getMedianWallAngle() * ALLIANCE_COLOR.getMultiplier();
                    double adjustment = robot.turret.limelightInterplut.get(offset);

                    Pose2d adjustedGoal;
                    if (adjustment < 0) {
                        adjustedGoal = new Pose2d(GOAL_POSE().getX() - (adjustment * ALLIANCE_COLOR.getMultiplier()), GOAL_POSE().getY(), GOAL_POSE().getHeading());
                    } else {
                        adjustedGoal = new Pose2d(GOAL_POSE().getX(), GOAL_POSE().getY() - adjustment, GOAL_POSE().getHeading());
                    }
                    telemetryData.addData("offset", offset);
                    telemetryData.addData("adjusted goal", adjustedGoal);
                } catch (Exception ignored) {
                    telemetryData.addData("tY Offset", "out of bounds error");
                }
                telemetryData.addData("bot pose", lastKnownPose);
                telemetryData.addData("Distance (m)", Constants.GOAL_POSE().minus(lastKnownPose).getTranslation().getNorm() * DistanceUnit.mPerInch);
            } else {
                telemetryData.addData("Median Y", "medianTy is Empty");
            }
        }

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        robot.updateLoop(telemetryData);
    }

    @Override
    public void end() {
//        Constants.END_POSE = robot.drive.getPose();
    }
}