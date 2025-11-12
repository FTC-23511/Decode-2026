package org.firstinspires.ftc.teamcode.tuning.motor;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp(name = "LaunchMotorTuner", group = "Motor")
public class LaunchMotorTuner extends CommandOpMode {
    public static double P = 0.004;
    public static double I = 0;
    public static double D = 0.000;
    public static double F = 0.00055;

    public static double TARGET_VEL = 0.0;
    public static double POS_TOLERANCE = 0;

    private static final PIDFController launcherPIDF = new PIDFController(P, I, D, F);

    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
    public ElapsedTime timer;
    private final Robot robot = Robot.getInstance();

    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;

    @Override
    public void initialize() {
        // Must have for all opModes
        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftLaunchMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightLaunchMotor");

        launcherPIDF.setTolerance(POS_TOLERANCE, 0);

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

        double motorVel = robot.launchEncoder.getCorrectedVelocity();
        double voltage = robot.getVoltage();

        launcherPIDF.setPIDF(P, I, D, F / (voltage / 12.0));

        launcherPIDF.setTolerance(POS_TOLERANCE, 0);
        launcherPIDF.setSetPoint(TARGET_VEL);

        double power = launcherPIDF.calculate(motorVel, TARGET_VEL);

        robot.launchMotors.set(power);

        telemetryData.addData("Loop Time", timer.milliseconds());
        timer.reset();

        telemetryData.addData("power", power);
        telemetryData.addData("voltage", voltage);
        telemetryData.addData("target velocity", TARGET_VEL);
        telemetryData.addData("actual velocity", motorVel);
        telemetryData.addData("sdk topMotor velocity", leftMotor.getVelocity());
        telemetryData.addData("sdk bottomMotor velocity", rightMotor.getVelocity());
        telemetryData.addData("encoder position", robot.launchEncoder.getPosition());

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        super.run();
        robot.pinpoint.update();
        telemetryData.update();
//        robot.controlHub.clearBulkCache();
//        robot.expansionHub.clearBulkCache();
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