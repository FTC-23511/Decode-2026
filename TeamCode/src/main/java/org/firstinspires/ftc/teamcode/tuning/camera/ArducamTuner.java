package org.firstinspires.ftc.teamcode.tuning.camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.TelemetryEx;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@TeleOp(name = "Arducam Tuner", group = "Tuning")
public class ArducamTuner extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private GamepadEx driver;
    private ElapsedTime timer;
    private final TelemetryEx telemetryEx = new TelemetryEx(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private long currentExposure = 15;
    private int currentGain = 100;

    @Override
    public void initialize() {
        // Must have for all opModes
        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;
        Constants.TESTING_OP_MODE = false; // Ensure periodic() runs for recordReadings

        // Resets the command scheduler
        super.reset();

        // Initialize the robot
        robot.init(hardwareMap);

        driver = new GamepadEx(gamepad1);

        // Toggle camera readings
        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> robot.camera.setRecordReadings(!robot.camera.isRecordReadingsEnabled()))
        );

        // Hard relocalization
        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> robot.camera.relocalizeArducam(true))
        );

        // Exposure Tuning (D-Pad Up/Down)
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> {
                    currentExposure++;
                    robot.camera.setCameraExposure(currentExposure);
                })
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> {
                    if (currentExposure > 1) {
                        currentExposure--;
                        robot.camera.setCameraExposure(currentExposure);
                    }
                })
        );

        // Gain Tuning (D-Pad Right/Left)
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> {
                    currentGain += 10;
                    robot.camera.setCameraGain(currentGain);
                })
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(() -> {
                    if (currentGain >= 10) {
                        currentGain -= 10;
                        robot.camera.setCameraGain(currentGain);
                    }
                })
        );
    }

    @Override
    public void run() {
        // Initialize movement-related hardware on first run
        if (timer == null) {
            robot.initHasMovement();
            timer = new ElapsedTime();
        }

        // Output telemetry as required
        telemetryEx.addData("Camera Readings", robot.camera.isRecordReadingsEnabled() ? "ON" : "OFF");
        telemetryEx.addData("Current Exposure (ms)", currentExposure);
        telemetryEx.addData("Current Gain", currentGain);

        Pose2d currentPose = robot.drive.getPose();
        telemetryEx.addData("Pose X", currentPose.getX());
        telemetryEx.addData("Pose Y", currentPose.getY());
        telemetryEx.addData("Pose Heading", Math.toDegrees(currentPose.getHeading()));

        int numDetections = (robot.camera.detections != null) ? robot.camera.detections.size() : 0;
        telemetryEx.addData("AprilTag Detections", numDetections);

        // Subsystem specific telemetry
        robot.camera.updateCameraTelemetry(telemetryEx);

        // Runs the command scheduler and updates telemetry
        robot.updateLoop(telemetryEx);
    }
}
