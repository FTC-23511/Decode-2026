package org.firstinspires.ftc.teamcode.tuning.subsystem;

import static org.firstinspires.ftc.teamcode.globals.Constants.BACKBOARD_Y_OFFSET;
import static org.firstinspires.ftc.teamcode.globals.Constants.GRAVITY;
import static org.firstinspires.ftc.teamcode.globals.Constants.LAUNCHER_HEIGHT;
import static org.firstinspires.ftc.teamcode.globals.Constants.TARGET_HEIGHT;
import static org.firstinspires.ftc.teamcode.globals.Constants.TESTING_OP_MODE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.util.TelemetryEx;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.MathFunctions;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp(name = "MathLaunchTuner", group = "Subsystem")
public class MathLaunchTuner extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;

    public static double TURRET_POS = 0.0; // raw servo pos
    public static boolean USE_RAW_SERVO_POS = false;
    public static double HOOD_SERVO_OUTPUT = 0.0; // either raw servo pos or hood angle
    public static double LAUNCHER_TARGET_VEL = 0.0; // ticks/sec
//    public static boolean TRANSFER_ON = false;
    public static double DISTANCE = 1.5; // meters
    public static boolean USE_LEGACY = true; // meters
    public static Intake.MotorState motorState = Intake.MotorState.STOP;

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
    }

    @Override
    public void run() {
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            robot.initHasMovement();
            timer = new ElapsedTime();
        }

        robot.intake.setIntake(motorState);

        robot.launcher.setRamp(true);

        if (USE_RAW_SERVO_POS) {
            HOOD_SERVO_OUTPUT = Range.clip(HOOD_SERVO_OUTPUT, 0.0, 1.0);
            robot.hoodServo.set(HOOD_SERVO_OUTPUT);
        } else {
            robot.launcher.setHood(HOOD_SERVO_OUTPUT);
        }

        robot.turret.setTurret(Turret.TurretState.ANGLE_CONTROL, TURRET_POS);

        robot.launcher.setFlywheelTicks(LAUNCHER_TARGET_VEL);
//        robot.launcher.setTransfer(TRANSFER_ON);

        telemetryEx.addData("Loop Time", timer.milliseconds());
        timer.reset();

        if (USE_LEGACY) {
            telemetryEx.addData("Math Output Required Ball Vel", MathFunctions.legacyDistanceToLauncherValues(DISTANCE)[0]);
            telemetryEx.addData("Math Output Required Hood Angle", MathFunctions.legacyDistanceToLauncherValues(DISTANCE)[1]);
        } else {
            telemetryEx.addData("Math Output Required Ball Vel", MathFunctions.distanceToLauncherValues(DISTANCE, Double.NaN)[0]);
            telemetryEx.addData("Math Output Required Hood Angle", MathFunctions.distanceToLauncherValues(DISTANCE, Double.NaN)[1]);
            telemetryEx.addData("Suggested Ball Vel for angle+distance", MathFunctions.calculateVelocity(DISTANCE, TARGET_HEIGHT + BACKBOARD_Y_OFFSET - LAUNCHER_HEIGHT, 90 - HOOD_SERVO_OUTPUT, GRAVITY));
            telemetryEx.addData("Suggested Ticks Vel for angle+distance", Launcher.launcherLUT.get(MathFunctions.calculateVelocity(DISTANCE, TARGET_HEIGHT + BACKBOARD_Y_OFFSET - LAUNCHER_HEIGHT, 90 - HOOD_SERVO_OUTPUT, GRAVITY)));
        }

        telemetryEx.addData("HOOD_SERVO_OUTPUT", HOOD_SERVO_OUTPUT);
        telemetryEx.addData("Hood Pos", robot.hoodServo.get());
        telemetryEx.addData("Launch Motor Power", robot.launchMotors.get());
        telemetryEx.addData("Actual Motor Vel", robot.launchEncoder.getCorrectedVelocity());
        telemetryEx.addData("Target Motor Vel", LAUNCHER_TARGET_VEL);
        telemetryEx.addData("Turret Encoder Pos", robot.turret.getRelativePos());

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        robot.updateLoop(telemetryEx);
        robot.launcher.periodic();
    }
}