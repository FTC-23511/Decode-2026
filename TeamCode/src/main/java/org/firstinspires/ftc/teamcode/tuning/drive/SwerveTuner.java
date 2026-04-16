package org.firstinspires.ftc.teamcode.tuning.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.util.TelemetryEx;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp(name = "SwerveTuner", group = "Drive")
public class SwerveTuner extends CommandOpMode {
    public static double FL_SERVO_POWER = 0.0;
    public static double FR_SERVO_POWER = 0.0;
    public static double BL_SERVO_POWER = 0.0;
    public static double BR_SERVO_POWER = 0.0;

    public static double FL_MOTOR_POWER = 0.0;
    public static double FR_MOTOR_POWER = 0.0;
    public static double BL_MOTOR_POWER = 0.0;
    public static double BR_MOTOR_POWER = 0.0;

    private final Robot robot = Robot.getInstance();
    private TelemetryEx telemetryEx;

    // We use Math.PI * 2 instead of 6.28 for higher precision
    private final double FULL_CIRCLE = Math.PI * 2;

    @Override
    public void initialize() {
        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;
        Constants.TESTING_OP_MODE = true;

        // Resets the command scheduler
        super.reset();

        // Initialize the robot
        robot.init(hardwareMap);

        telemetryEx = new TelemetryEx(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
    }

    @Override
    public void run() {
        robot.FLmotor.set(FL_MOTOR_POWER);
        robot.FRmotor.set(FR_MOTOR_POWER);
        robot.BLmotor.set(BL_MOTOR_POWER);
        robot.BRmotor.set(BR_MOTOR_POWER);

        robot.FLswervo.set(FL_SERVO_POWER);
        robot.FRswervo.set(FR_SERVO_POWER);
        robot.BLswervo.set(BL_SERVO_POWER);
        robot.BRswervo.set(BR_SERVO_POWER);

        // 1. Get current positions
        double flPos = robot.FLswervo.getAbsoluteEncoder().getCurrentPosition();
        double frPos = robot.FRswervo.getAbsoluteEncoder().getCurrentPosition();
        double blPos = robot.BLswervo.getAbsoluteEncoder().getCurrentPosition();
        double brPos = robot.BRswervo.getAbsoluteEncoder().getCurrentPosition();

        // 2. Calculate the new offsets (Full Circle - Current Position)
        // We use % FULL_CIRCLE to ensure the offset stays cleanly within the 0 to 2PI range
        double calcFlOffset = (FULL_CIRCLE - flPos) % FULL_CIRCLE;
        double calcFrOffset = (FULL_CIRCLE - frPos) % FULL_CIRCLE;
        double calcBlOffset = (FULL_CIRCLE - blPos) % FULL_CIRCLE;
        double calcBrOffset = (FULL_CIRCLE - brPos) % FULL_CIRCLE;

        // 3. Telemetry out the raw positions
        telemetryEx.addData("FL Swervo Abs Pos", flPos);
        telemetryEx.addData("FR Swervo Abs Pos", frPos);
        telemetryEx.addData("BL Swervo Abs Pos", blPos);
        telemetryEx.addData("BR Swervo Abs Pos", brPos);

        telemetryEx.addData("-----------------------", "");

        // 4. Telemetry out the calculated offsets
        telemetryEx.addData("-> NEW FL_ENCODER_OFFSET", calcFlOffset);
        telemetryEx.addData("-> NEW FR_ENCODER_OFFSET", calcFrOffset);
        telemetryEx.addData("-> NEW BL_ENCODER_OFFSET", calcBlOffset);
        telemetryEx.addData("-> NEW BR_ENCODER_OFFSET", calcBrOffset);

        // DO NOT REMOVE: Runs command scheduler, updates telemetry, and clears bulk cache
        robot.updateLoop(telemetryEx);
    }
}