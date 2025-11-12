package org.firstinspires.ftc.teamcode.tuning.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp(name = "LauncherSubsystemTuner", group = "Subsystem")
public class LauncherSubsystemTuner extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;

    public static boolean USE_RAW_SERVO_POS = false;
    public static double SERVO_OUTPUT = 0.0; // either raw servo pos or hood angle
    public static double TARGET_VEL = 0.0; // ticks/sec
    public static double DISTANCE = 1.5; // meters

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
    }

    @Override
    public void run() {
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            robot.initHasMovement();
            timer = new ElapsedTime();
        }

        robot.launcher.setRamp(true);

        if (USE_RAW_SERVO_POS) {
            SERVO_OUTPUT = Range.clip(SERVO_OUTPUT, 0.0, 1.0);
            robot.hoodServo.set(SERVO_OUTPUT);
        } else {
            robot.launcher.setHood(SERVO_OUTPUT);
        }

        robot.launcher.setFlywheelTicks(TARGET_VEL);

        telemetryData.addData("Loop Time", timer.milliseconds());
        timer.reset();

        telemetryData.addData("Math Output Required Ball Vel", Launcher.distanceToLauncherValues(DISTANCE)[0]);
        telemetryData.addData("Math Output Required Hood Angle", Launcher.distanceToLauncherValues(DISTANCE)[1]);
        telemetryData.addData("SERVO_OUTPUT", SERVO_OUTPUT);
        telemetryData.addData("Hood Pos", robot.hoodServo.get());
        telemetryData.addData("Motor Power", robot.launchMotors.get());
        telemetryData.addData("Motor Velocity", robot.launchEncoder.getCorrectedVelocity());
        telemetryData.addData("TARGET_VEL", TARGET_VEL);

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        super.run();
        telemetryData.update();
//        robot.controlHub.clearBulkCache();
//        robot.expansionHub.clearBulkCache();
    }
}