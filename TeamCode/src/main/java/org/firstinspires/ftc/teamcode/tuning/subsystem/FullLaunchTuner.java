package org.firstinspires.ftc.teamcode.tuning.subsystem;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.TelemetryEx;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.MathFunctions;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp(name = "FullLaunchTuner", group = "Subsystem")
public class FullLaunchTuner extends CommandOpMode {
    public ElapsedTime timer;

    public static double INTAKE_MOTOR_POWER = 0.0;

    public static double LAUNCHER_P = 0.000;
    public static double LAUNCHER_I = 0.0;
    public static double LAUNCHER_D = 0.0;
    public static double LAUNCHER_F = 0.0000;

    public static double LAUNCHER_TARGET_VEL = 0.0;
    public static boolean TRANSFER_ON = false;

    public static double HOOD_SERVO_POS = 0.0;
    public static double TURRET_RADIANS = 0.0;
    public static double RAMP_SERVO_POS = 0.0;

    private final PIDFController launcherPIDF = new PIDFController(LAUNCHER_P, LAUNCHER_I, LAUNCHER_D, LAUNCHER_F);

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

        robot.turret.setTurret(Turret.TurretState.GOAL_LOCK_CONTROL, 0);
    }

    @Override
    public void run() {
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            robot.initHasMovement();
            timer = new ElapsedTime();
        }

        if (gamepad1.circleWasPressed()) {
            INTAKE_MOTOR_POWER = 0.0;
            LAUNCHER_TARGET_VEL = 1700;
            RAMP_SERVO_POS = RAMP_DISENGAGED;
            HOOD_SERVO_POS = MAX_HOOD_SERVO_POS;
        }

        if (gamepad1.triangleWasPressed()) {
            INTAKE_MOTOR_POWER = 1.0;
            RAMP_SERVO_POS = RAMP_ENGAGED;
        }

        INTAKE_MOTOR_POWER = Range.clip(INTAKE_MOTOR_POWER, -1.0, 1.0);
        robot.intakeMotor.set(INTAKE_MOTOR_POWER);

        HOOD_SERVO_POS = Range.clip(HOOD_SERVO_POS, MIN_HOOD_SERVO_POS, MAX_HOOD_SERVO_POS);
        robot.hoodServo.set(HOOD_SERVO_POS);

        RAMP_SERVO_POS = Range.clip(RAMP_SERVO_POS, RAMP_DISENGAGED, RAMP_ENGAGED);
        robot.rampServo.set(RAMP_SERVO_POS);

//        STOPPER_SERVO_POS = Range.clip(STOPPER_SERVO_POS, STOPPER_DISENGAGED_POS, STOPPER_ENGAGED_POS);
//        robot.stopperServo.set(STOPPER_SERVO_POS);

        launcherPIDF.setPIDF(LAUNCHER_P, LAUNCHER_I, LAUNCHER_D, LAUNCHER_F);
        launcherPIDF.setSetPoint(LAUNCHER_TARGET_VEL);

        double launcherVel = robot.launchEncoder.getCorrectedVelocity();
        double launcherPower = launcherPIDF.calculate(launcherVel);

        robot.launchMotors.set(launcherPower);
        robot.launcher.setTransfer(TRANSFER_ON);

        robot.turretServos.set(MathFunctions.convertRadianToServoPos(TURRET_RADIANS) + TURRET_SERVO_OFFSET);

        telemetryEx.addData("Loop Time", timer.milliseconds());
        timer.reset();

        telemetryEx.addData("launcherPower", launcherPower);
        telemetryEx.addData("launcher target velocity", LAUNCHER_TARGET_VEL);
        telemetryEx.addData("launcher actual velocity", launcherVel);

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        robot.updateLoop(telemetryEx);
    }
}
