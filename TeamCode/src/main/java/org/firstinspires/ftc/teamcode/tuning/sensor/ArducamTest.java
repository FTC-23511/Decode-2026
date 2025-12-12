package org.firstinspires.ftc.teamcode.tuning.sensor;


import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.MathUtils;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "ArducamTest", group = "Sensor")
public class ArducamTest extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;

    public static double angleVal = 0.0;

    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();

    @Override
    public void initialize() {
        // Must have for all opModes
        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;
        Constants.TESTING_OP_MODE = true;

        // Resets the command scheduler
        super.reset();

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> angleVal += 0.25),
                        new InstantCommand(() -> robot.turret.setTurret(Turret.TurretState.ANGLE_CONTROL, angleVal))
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> angleVal -= 0.25),
                        new InstantCommand(() -> robot.turret.setTurret(Turret.TurretState.ANGLE_CONTROL, angleVal))
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> robot.turret.setTurret(Turret.TurretState.OFF, 0))
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(() -> robot.turret.setTurret(Turret.TurretState.TX_CONTROL, 0))
        );
    }

    @Override
    public void initialize_loop() {
        if (timer == null) {
            robot.initHasMovement();
            timer = new ElapsedTime();
        }

        robot.camera.updateCameraResult(3);
        robot.camera.getCameraTelemetry(telemetry);

        telemetryData.addData("Loop Time", timer.milliseconds());

        Pose2d cameraPose = robot.camera.getCameraPose();
        double[] targetDegrees = robot.camera.getTargetDegrees();

        telemetryData.addData("camera pose", cameraPose == null ? "null" : cameraPose.toString());
        telemetryData.addData("tX", targetDegrees == null ? "null" : targetDegrees[0]);
        telemetryData.addData("tX Offset", targetDegrees == null ? "null" : robot.camera.getTxOffset(robot.turret.getTurretPose()));
        telemetryData.addData("tY", targetDegrees == null ? "null" : targetDegrees[1]);
        telemetryData.addData("distance", APRILTAG_POSE().minus(robot.drive.getPose()).getTranslation().getNorm());
        telemetryData.addData("test distance", TEST_DISTANCE);
        telemetryData.addData("tag height (px)", robot.camera.getTagHeight());
        telemetryData.addData("roi cameraH (px)", robot.camera.cameraH);
        telemetryData.addData("roi cameraY (px)", robot.camera.cameraY);
        telemetryData.addData("turret state", Turret.turretState);
        timer.reset();

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        telemetryData.update();
        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();
        robot.pinpoint.update();
    }

    @Override
    public void run() {
        if (!Turret.turretState.equals(Turret.TurretState.TX_CONTROL)) {
            robot.camera.updateCameraResult(3);
            robot.camera.getCameraTelemetry(telemetry);
        }

        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            robot.initHasMovement();
            timer = new ElapsedTime();
        }

        telemetryData.addData("Loop Time", timer.milliseconds());

        Pose2d cameraPose = robot.camera.getCameraPose();
        double[] targetDegrees = robot.camera.getTargetDegrees();

        telemetryData.addData("camera pose", cameraPose == null ? "null" : cameraPose.toString());
        telemetryData.addData("tX", targetDegrees == null ? "null" : targetDegrees[0]);
        telemetryData.addData("tX Offset", targetDegrees == null ? "null" : robot.camera.getTxOffset(robot.turret.getTurretPose()));
        telemetryData.addData("tY", targetDegrees == null ? "null" : targetDegrees[1]);
        telemetryData.addData("distance", APRILTAG_POSE().minus(robot.drive.getPose()).getTranslation().getNorm());
        telemetryData.addData("test distance", TEST_DISTANCE);
        telemetryData.addData("tag height (px)", robot.camera.getTagHeight());
        telemetryData.addData("roi cameraH (px)", robot.camera.cameraH);
        telemetryData.addData("roi cameraY (px)", robot.camera.cameraY);
        timer.reset();

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        CommandScheduler.getInstance().run();
        telemetryData.update();
        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();
        robot.pinpoint.update();
    }

    @Override
    public void end() {
//        Constants.END_POSE = robot.drive.getPose();
    }
}