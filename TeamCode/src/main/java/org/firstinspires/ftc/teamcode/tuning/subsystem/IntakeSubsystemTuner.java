package org.firstinspires.ftc.teamcode.tuning.subsystem;

import static org.firstinspires.ftc.teamcode.globals.Constants.BACK_DISTANCE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.globals.Constants.FRONT_DISTANCE_THRESHOLD;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp(name = "IntakeSubsystemTuner", group = "Subsystem")
public class IntakeSubsystemTuner extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;

    public static double SERVO_POS = 0.0;
    public static double MOTOR_POWER = 0.0;

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

        SERVO_POS = Range.clip(SERVO_POS, 0.0, 1.0);
        robot.intakePivotServo.set(SERVO_POS);

        MOTOR_POWER = Range.clip(MOTOR_POWER, -1.0, 1.0);
        robot.intakeMotors.set(MOTOR_POWER);

        telemetryData.addData("Loop Time", timer.milliseconds());
        timer.reset();

        telemetryData.addData("SERVO_POS", SERVO_POS);
        telemetryData.addData("MOTOR_POWER", MOTOR_POWER);

        telemetryData.addData("Front Threshold", FRONT_DISTANCE_THRESHOLD);
        telemetryData.addData("Back Threshold", BACK_DISTANCE_THRESHOLD);

        telemetryData.addData("Front Threshold Met", robot.frontDistanceSensor.isActive());
        telemetryData.addData("Back Threshold Met", robot.backDistanceSensor.isActive());

        telemetryData.addData("Robot Current", robot.getVoltage());
        telemetryData.addData("Intake Current", ((MotorEx) robot.intakeMotors.getMotor()).getCurrent(CurrentUnit.MILLIAMPS));
        telemetryData.addData("Intake Over Current", ((MotorEx) robot.intakeMotors.getMotor()).isOverCurrent());

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        robot.updateLoop(telemetryData);
    }
}