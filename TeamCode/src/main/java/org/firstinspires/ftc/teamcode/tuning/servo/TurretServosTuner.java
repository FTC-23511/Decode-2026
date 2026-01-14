package org.firstinspires.ftc.teamcode.tuning.servo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.MathUtils;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp(name = "TurretServosTuner", group = "Servo")
public class TurretServosTuner extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public static double TARGET_POS = 0.0;
    public static double TARGET_VEL = 0.0;
    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
    public ElapsedTime timer;
    private final Robot robot = Robot.getInstance();

    @Override
    public void initialize() {
        // Must have for all opModes
        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;
        Constants.TESTING_OP_MODE = true;
        Turret.turretState = Turret.TurretState.GOAL_LOCK_CONTROL;

        // Resets the command scheduler
        super.reset();

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> robot.turret.setTurret(Turret.TurretState.GOAL_LOCK_CONTROL, 0))
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(() -> robot.turret.setTurret(Turret.TurretState.ANGLE_CONTROL, TARGET_POS))
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> robot.turret.setTurret(Turret.TurretState.OFF, 0))
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> robot.turret.resetTurretEncoder())
        );
    }

    @Override
    public void run() {
        if (timer == null) {
            robot.initHasMovement();
            robot.turret.setTurret(Turret.TurretState.GOAL_LOCK_CONTROL, 0);
            timer = new ElapsedTime();
        }
        robot.turret.updateCoefficients(); // this internally updates all the coefficients, do not remove

        // update pos and vel targets
        robot.turret.turretController.setSetPoint(TARGET_POS);
        Turret.targetVel = TARGET_VEL;

        telemetryData.addData("Loop Time", timer.milliseconds());
        timer.reset();

        telemetryData.addData("Raw Pos", robot.turretEncoder.getPosition());
        telemetryData.addData("Analog Pos", MathUtils.normalizeRadians(robot.analogTurretEncoder.getCurrentPosition(), false));

        telemetryData.addData("Actual Pos", robot.turret.getPosition());
        telemetryData.addData("Target Pos", TARGET_POS);

        telemetryData.addData("Target Vel", TARGET_VEL);
        telemetryData.addData("Unfiltered Vel", -robot.turret.turretController.getVelocityError()); // TODO: check if this is actually equivalent to unfiltered vel

        telemetryData.addData("Get Power", robot.turretServos.getSpeeds().toString());
        telemetryData.addData("atSetPoint", robot.turret.turretController.atSetPoint());

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        robot.updateLoop(telemetryData);
    }
    
//    @Override
//    public void end() {
//        Log.v("P", String.valueOf(P));
//        Log.v("I", String.valueOf(I));
//        Log.v("D", String.valueOf(D));
//        Log.v("F", String.valueOf(F));
//        Log.v("posTolerance", String.valueOf(POS_TOLERANCE));
//    }
}