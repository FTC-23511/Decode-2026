package org.firstinspires.ftc.teamcode.tuning.sensor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.UninterruptibleCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.MathUtils;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commandbase.commands.CancelCommand;
import org.firstinspires.ftc.teamcode.commandbase.commands.StationaryAimbotFullLaunch;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp(name = "CameraTuner", group = "Sensor")
public class CameraTuner extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;

    private final TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();

    public static double P = 0.00;
    public static double I = 0;
    public static double D = 0.000;
    public static double F = 0.000;
    public static double MIN_OUTPUT = 0.15;

    public static double TARGET_POS = 0.0;
    public static double POS_TOLERANCE = 0.03;

    public static double angleVal = 0;

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

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                new StationaryAimbotFullLaunch()
        );

        driver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                new UninterruptibleCommand(new CancelCommand())
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

        telemetryData.addData("Turret Set Point", robot.turret.turretController.getSetPoint());
        telemetryData.addData("Turret Enum", Turret.turretState);
        telemetryData.addData("Angle Mode Value", angleVal);

        double robotPose = robot.drive.getPose().getRotation().getDegrees();
        double turretPos = Math.toDegrees(robot.turret.getPosition());

        telemetryData.addData("Robot Pose", robotPose);
        telemetryData.addData("turretPos", turretPos);

        telemetryData.addData("Robot Full Angle", (
                robotPose + turretPos - 90.0)
        );
        telemetryData.addData("Robot Full Corrected Angle", MathUtils.normalizeDegrees(
                robotPose + turretPos - 90.0, false)
        );

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        robot.updateLoop(telemetryData);
    }

    @Override
    public void end() {
//        Constants.END_POSE = robot.drive.getPose();
    }
}