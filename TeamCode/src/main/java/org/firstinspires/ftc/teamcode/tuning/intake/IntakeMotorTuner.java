package org.firstinspires.ftc.teamcode.tuning.intake;

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

        MOTOR_POWER = Range.clip(MOTOR_POWER, -1.0, 1.0);
        robot.intakeMotor.set(MOTOR_POWER);

        telemetryEx.addData("Loop Time", timer.milliseconds());
        timer.reset();

        telemetryEx.addData("MOTOR_POWER", MOTOR_POWER);
        telemetryEx.addData("MOTOR_CURRENT", ((MotorEx) robot.intakeMotor.getMotor()).getCurrent(CurrentUnit.MILLIAMPS));

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        super.run();
        telemetryEx.update();
    }
}