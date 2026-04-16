package org.firstinspires.ftc.teamcode.tuning.launcher;
import static org.firstinspires.ftc.teamcode.globals.Constants.TESTING_OP_MODE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.util.TelemetryEx;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp(name = "LauncherMotorTuner", group = "Launcher")
public class LauncherMotorTuner extends CommandOpMode {
    public ElapsedTime timer;

    public static double P = 0.000;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 0.0000;

    public static double LAUNCHER_TARGET_VEL = 0.0;

    public static boolean REVERSE_ENCODER = false;
    public static boolean REVERSE_MOTOR = false;

    private double MAX_VELO = 0;

    private final PIDFController launcherPIDF = new PIDFController(P, I, D, F);

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
    }

    @Override
    public void run() {
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            robot.initHasMovement();
            timer = new ElapsedTime();
        }

        launcherPIDF.setPIDF(P, I, D, F);
        launcherPIDF.setSetPoint(LAUNCHER_TARGET_VEL);

        double launcherVel = robot.launchEncoder.getCorrectedVelocity();
        double launcherPower = launcherPIDF.calculate(launcherVel) * (REVERSE_MOTOR ? -1 : 1);

        if (launcherVel > MAX_VELO) {
            MAX_VELO = launcherVel;
        }

        robot.launchEncoder.setDirection(REVERSE_ENCODER ? Motor.Direction.REVERSE : Motor.Direction.FORWARD);
        robot.launchMotors.set(launcherPower);

        telemetryEx.addData("Loop Time", timer.milliseconds());
        timer.reset();

        telemetryEx.addData("launcherPower", launcherPower);
        telemetryEx.addData("launcher pos", robot.launchEncoder.getPosition());
        telemetryEx.addData("launcher target velocity", LAUNCHER_TARGET_VEL);
        telemetryEx.addData("MAX_VELO", MAX_VELO);
        telemetryEx.addData("launcher actual velocity", launcherVel);

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        robot.updateLoop(telemetryEx);
    }
}
