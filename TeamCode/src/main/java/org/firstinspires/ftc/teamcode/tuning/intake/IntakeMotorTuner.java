package org.firstinspires.ftc.teamcode.tuning.intake;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;
import static com.qualcomm.robotcore.hardware.Gamepad.RUMBLE_DURATION_CONTINUOUS;
import static org.firstinspires.ftc.teamcode.globals.Constants.INTAKE_CURRENT_BUFFER_TIME;
import static org.firstinspires.ftc.teamcode.globals.Constants.TESTING_OP_MODE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.TelemetryEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp(name = "IntakeMotorTuner", group = "Intake")
public class IntakeMotorTuner extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;

    public static double INTAKE_MOTOR_POWER = 0.0;
    public static double TRANSFER_MOTOR_POWER = 0.0;
    public static double BOTH_MOTOR_POWER = 0.0;
    public static boolean USE_BOTH = true;

    TelemetryEx telemetryEx = new TelemetryEx(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();

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
        // Reset heading
        driver.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new InstantCommand(() -> {
                    robot.intake.setIntake(Intake.MotorState.FORWARD);
                    TRANSFER_MOTOR_POWER = Constants.INTAKE_FORWARD_SPEED;
                })
        );

        driver.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new InstantCommand(() -> {
                    robot.intake.setIntake(Intake.MotorState.STOP);
                    TRANSFER_MOTOR_POWER = 0.0;
                })
        );

        driver.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new InstantCommand(() -> {
                    robot.intake.setIntake(Intake.MotorState.REVERSE);
                    TRANSFER_MOTOR_POWER = Constants.INTAKE_REVERSE_SPEED;
                })
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

        robot.intake.updateDistanceSensor();

        if (robot.intake.currentBuffered) {
            robot.intake.updateCurrentSensor();
        } else if (robot.intake.currentTimer.milliseconds() > INTAKE_CURRENT_BUFFER_TIME) {
            robot.intake.currentTimer.reset();
            robot.intake.currentBuffered = true;
        } else {
            robot.intake.withinCurrent = false;
        }

        if (USE_BOTH) {
            robot.transferMotor.set(BOTH_MOTOR_POWER);
            robot.intakeMotor.set(BOTH_MOTOR_POWER);
        } else {
            robot.transferMotor.set(TRANSFER_MOTOR_POWER);
            robot.intakeMotor.set(INTAKE_MOTOR_POWER);
        }

        if (!robot.intake.isRumbling && robot.intake.transferFull()) {
            gamepad1.rumble(RUMBLE_DURATION_CONTINUOUS);
            gamepad1.setLedColor(255, 0, 0, LED_DURATION_CONTINUOUS);
            robot.intake.isRumbling = true;
        } else if (robot.intake.isRumbling) {
            gamepad1.stopRumble();
            gamepad1.setLedColor(0, 0, 255, LED_DURATION_CONTINUOUS);
        }

        telemetryEx.addData("Loop Time", timer.milliseconds());
        timer.reset();

        telemetryEx.addData("Intake Current", ((MotorEx) robot.intakeMotor.getMotor()).getCurrent(CurrentUnit.MILLIAMPS));
        telemetryEx.addData("Intake Overcurrent", ((MotorEx) robot.intakeMotor.getMotor()).isOverCurrent());
        telemetryEx.addData("Distance (cm)", robot.intake.getDistance());
        telemetryEx.addData("Transfer Full", robot.intake.transferFull());

        telemetryEx.addData("Within Current", robot.intake.withinCurrent);
        telemetryEx.addData("Current Timer", robot.intake.currentTimer.milliseconds());

        telemetryEx.addData("Within Distance", robot.intake.withinDistance);
        telemetryEx.addData("Distance Timer", robot.intake.distanceTimer.milliseconds());

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        robot.updateLoop(telemetryEx);
    }
}