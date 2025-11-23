package org.firstinspires.ftc.teamcode.tuning.servo;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.MathUtils;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp(name = "TurretServosTuner", group = "Servo")
public class TurretServosTuner extends CommandOpMode {
    public static double P = 0.00;
    public static double I = 0;
    public static double D = 0.000;
    public static double F = 0.000;
    public static double MIN_OUTPUT = 0.15;

    public static double TARGET_POS = 0.0;
    public static double POS_TOLERANCE = 0.03;
    public static double MIN_INTEGRAL = 0.0;
    public static double MAX_INTEGRAL = 1.0;

    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
    public ElapsedTime timer;
    private final Robot robot = Robot.getInstance();

    @Override
    public void initialize() {
        // Must have for all opModes
        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;
        Turret.turretState = Turret.TurretState.OFF;

        // Resets the command scheduler
        super.reset();

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);
    }

    @Override
    public void run() {
        if (timer == null) {
            Turret.turretState = Turret.TurretState.OFF;
            timer = new ElapsedTime();
        }
        Turret.turretState = Turret.TurretState.OFF;
        double servoPos = robot.turret.getPosition();

        robot.turret.turretController.setPIDF(P, I, D, F);
        robot.turret.turretController.setTolerance(POS_TOLERANCE);
        robot.turret.turretController.setMinimumOutput(MIN_OUTPUT);
        robot.turret.turretController.setIntegrationBounds(MIN_INTEGRAL, MAX_INTEGRAL);

        double power = robot.turret.turretController.calculate(servoPos, TARGET_POS);

        if (robot.turret.turretController.atSetPoint()) {
            robot.turretServos.set(0);
            robot.turret.turretController.clearTotalError();
        } else {
            robot.turretServos.set(power);
        }

        telemetryData.addData("Loop Time", timer.milliseconds());
        timer.reset();

        telemetryData.addData("Actual Pos", servoPos);
        telemetryData.addData("Target Pos", TARGET_POS);

        telemetryData.addData("Set Power", power);
        telemetryData.addData("Get Power", robot.turretServos.getSpeeds().toString());

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        robot.updateLoop(telemetryData);
    }
    
    @Override
    public void end() {
        Log.v("P", String.valueOf(P));
        Log.v("I", String.valueOf(I));
        Log.v("D", String.valueOf(D));
        Log.v("F", String.valueOf(F));
        Log.v("posTolerance", String.valueOf(POS_TOLERANCE));
    }
}