package org.firstinspires.ftc.teamcode.tuning.intake;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;
import static com.qualcomm.robotcore.hardware.Gamepad.RUMBLE_DURATION_CONTINUOUS;
import static org.firstinspires.ftc.teamcode.globals.Constants.TESTING_OP_MODE;

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

    public static double MOTOR_POWER = 0.0;

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
                new InstantCommand(() -> robot.intake.setIntake(Intake.MotorState.FORWARD))
        );

        driver.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new InstantCommand(() -> robot.intake.setIntake(Intake.MotorState.STOP))
        );

        driver.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new InstantCommand(() -> robot.intake.setIntake(Intake.MotorState.REVERSE))
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

        if (!gamepad1.isRumbling() && Intake.motorState.equals(Intake.MotorState.FORWARD) && robot.intake.transferFull()) {
            gamepad1.rumble(RUMBLE_DURATION_CONTINUOUS);
            gamepad1.setLedColor(255, 0, 0, LED_DURATION_CONTINUOUS);
        } else if (gamepad1.isRumbling() && !Intake.motorState.equals(Intake.MotorState.FORWARD)) {
            gamepad1.stopRumble();
            gamepad1.setLedColor(0, 0, 255, LED_DURATION_CONTINUOUS);
        }

        telemetryEx.addData("Loop Time", timer.milliseconds());
        timer.reset();

        telemetryEx.addData("Intake Current", ((MotorEx) robot.intakeMotor.getMotor()).getCurrent(CurrentUnit.MILLIAMPS));
        telemetryEx.addData("Transfer Full", robot.intake.transferFull());
        telemetryEx.addData("Transfer Empty", robot.intake.transferEmpty());

        telemetryEx.addData("Within Current", robot.intake.withinCurrent);
        telemetryEx.addData("Current Timer", robot.intake.currentTimer.milliseconds());

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        robot.updateLoop(telemetryEx);
        robot.intake.periodic();
    }
}