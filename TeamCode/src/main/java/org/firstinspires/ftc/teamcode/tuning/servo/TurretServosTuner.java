package org.firstinspires.ftc.teamcode.tuning.servo;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.MathUtils;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp(name = "TurretServosTuner")
public class TurretServosTuner extends CommandOpMode {
    public static double P = 0.00;
    public static double I = 0;
    public static double D = 0.000;
    public static double F = 0.000;
    public static double MIN_OUTPUT = 0.15;

    public static double TARGET_POS = 0.0;
    public static double POS_TOLERANCE = 0.03;

    private static final PIDFController turretPIDF = new PIDFController(P, I, D, F);

    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
    public ElapsedTime timer;
    private final Robot robot = Robot.getInstance();

    @Override
    public void initialize() {
        // Must have for all opModes
        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;

        turretPIDF.setTolerance(POS_TOLERANCE, 0);

        // Resets the command scheduler
        super.reset();

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
        }

        double servoPos = robot.turret.getPosition();

        turretPIDF.setPIDF(P, I, D, F);
        turretPIDF.setTolerance(POS_TOLERANCE);
        turretPIDF.setMinimumOutput(MIN_OUTPUT);
        turretPIDF.setSetPoint(TARGET_POS);

        double power = turretPIDF.calculate(servoPos, TARGET_POS);

        robot.turretServos.set(power);

        telemetryData.addData("Loop Time", timer.milliseconds());
        timer.reset();

        telemetryData.addData("Actual Pos", servoPos);
        telemetryData.addData("Target Pos", TARGET_POS);

        telemetryData.addData("Set Power", power);
        telemetryData.addData("Get Power", robot.turretServos.getSpeeds().toString());

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        super.run();
        robot.pinpoint.update();
        telemetryData.update();
        robot.controlHub.clearBulkCache();
        robot.expansionHub.clearBulkCache();
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