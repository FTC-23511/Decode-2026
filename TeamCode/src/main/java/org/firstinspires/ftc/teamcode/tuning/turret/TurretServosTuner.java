package org.firstinspires.ftc.teamcode.tuning.turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.util.TelemetryEx;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.MathFunctions;
import org.firstinspires.ftc.teamcode.globals.Robot;
import static org.firstinspires.ftc.teamcode.globals.Constants.*;

@Config
@TeleOp(name = "TurretServosTuner", group = "Turret")
public class TurretServosTuner extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public static double TARGET_RADIANS = 0.0;
    TelemetryEx telemetryEx = new TelemetryEx(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
    public ElapsedTime timer;
    private final Robot robot = Robot.getInstance();

    @Override
    public void initialize() {
        // Must have for all opModes
        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;
        Constants.TESTING_OP_MODE = true;

        // Resets the command scheduler
        super.reset();

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);

        robot.turret.setTurret(Turret.TurretState.GOAL_LOCK_CONTROL, 0);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
        }

        robot.turretServos.set(MathFunctions.convertRadianToServoPos(TARGET_RADIANS) + TURRET_SERVO_OFFSET);

        telemetryEx.addData("TARGET POS (Radians)", TARGET_RADIANS);
        telemetryEx.addData("TARGET POS (Servo Pos)", robot.turretServos.get());
        telemetryEx.addData("Turret at pos", robot.turret.readyToLaunch());
        telemetryEx.addData("Analog pos", robot.turret.getAnalogPos());
        telemetryEx.addData("Relative pos", robot.turret.getRelativePos());
        robot.updateLoop(telemetryEx);

    }
}