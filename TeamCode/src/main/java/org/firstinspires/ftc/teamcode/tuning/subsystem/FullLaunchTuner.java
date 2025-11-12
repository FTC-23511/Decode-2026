package org.firstinspires.ftc.teamcode.tuning.subsystem;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp(name = "FullLaunchTuner", group = "Subsystem")
public class FullLaunchTuner extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;

    public static double INTAKE_SERVO_POS = INTAKE_PIVOT_FORWARD;
    public static double HOOD_SERVO_POS = MAX_HOOD_SERVO_POS;
    public static double RAMP_SERVO_POS = RAMP_ENGAGED;
    public static double INTAKE_MOTOR_POWER = 0;
    public static double LAUNCHER_MOTOR_POWER = 0;

    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();

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
        // Reset heading
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> robot.drive.setPose(new Pose2d()))
        );
    }

    @Override
    public void run() {
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            robot.initHasMovement();
            timer = new ElapsedTime();
        }

        robot.intakePivotServo.set(INTAKE_SERVO_POS);
        robot.hoodServo.set(HOOD_SERVO_POS);
        robot.rampServo.set(RAMP_SERVO_POS);

        robot.intakeMotors.set(INTAKE_MOTOR_POWER);
        robot.launchMotors.set(LAUNCHER_MOTOR_POWER);

        telemetryData.addData("Loop Time", timer.milliseconds());
        timer.reset();

        telemetryData.addData("INTAKE_SERVO_POS", INTAKE_SERVO_POS);
        telemetryData.addData("HOOD_SERVO_POS", HOOD_SERVO_POS);
        telemetryData.addData("RAMP_SERVO_POS", RAMP_SERVO_POS);

        telemetryData.addData("INTAKE_MOTOR_POWER", INTAKE_MOTOR_POWER);
        telemetryData.addData("LAUNCHER_MOTOR_POWER", LAUNCHER_MOTOR_POWER);
        telemetryData.addData("Launch Encoder Position", robot.launchMotors.getPositions().get(0));
        telemetryData.addData("Launch Velocity", robot.launchMotors.getVelocity());

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        super.run();
        telemetryData.update();
    }
}