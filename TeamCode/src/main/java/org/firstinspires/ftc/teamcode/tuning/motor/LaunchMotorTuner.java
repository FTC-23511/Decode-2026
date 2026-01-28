package org.firstinspires.ftc.teamcode.tuning.motor;

import static org.firstinspires.ftc.teamcode.globals.Constants.MAX_HOOD_ANGLE;
import static org.firstinspires.ftc.teamcode.globals.Constants.MAX_HOOD_SERVO_POS;
import static org.firstinspires.ftc.teamcode.globals.Constants.TESTING_OP_MODE;

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

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
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

    private double motorVel = 0;
    private double LEFT_MOTOR_POWER = 0;
    private double RIGHT_MOTOR_POWER = 0;

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
        TESTING_OP_MODE = true;

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftLaunchMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightLaunchMotor");

        robot.launcher.setHood(MAX_HOOD_ANGLE);

        launcherPIDF.setTolerance(POS_TOLERANCE, 0);

        // Resets the command scheduler
        super.reset();
    }

    @Override
    public void run() {
        if (timer == null) {
            robot.initHasMovement();
            timer = new ElapsedTime();
        }

        double newVel = robot.launchEncoder.getCorrectedVelocity();
        if (Math.abs(newVel) < Constants.LAUNCHER_MAX_VELOCITY) {
            motorVel = newVel;
        }

        launcherPIDF.setPIDF(P, I, D, F);

        launcherPIDF.setTolerance(POS_TOLERANCE, 0);
        launcherPIDF.setSetPoint(TARGET_VEL);

        double power = launcherPIDF.calculate(motorVel, TARGET_VEL);

        if (LEFT_MOTOR_POWER != 0) {
            leftMotor.setPower(LEFT_MOTOR_POWER);
        } else if (RIGHT_MOTOR_POWER != 0) {
            rightMotor.setPower(RIGHT_MOTOR_POWER);
        } else {
            robot.launchMotors.set(power);
        }

        telemetryData.addData("Loop Time", timer.milliseconds());
        timer.reset();

        telemetryData.addData("power", power);
        telemetryData.addData("target velocity", TARGET_VEL);
        telemetryData.addData("actual velocity", motorVel);
        telemetryData.addData("sdk topMotor velocity", leftMotor.getVelocity());
        telemetryData.addData("sdk bottomMotor velocity", rightMotor.getVelocity());
        telemetryData.addData("encoder position", robot.launchEncoder.getPosition());
        telemetryData.addData("raw vel", robot.launchEncoder.getRawVelocity());

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