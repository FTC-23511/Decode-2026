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
    public static double FL_SERVO_POS = 0.5;
    public static double FR_SERVO_POS = 0.5;
    public static double BL_SERVO_POS = 0.5;
    public static double BR_SERVO_POS = 0.5;

    public static double FL_MOTOR_POWER = 0.0;
    public static double FR_MOTOR_POWER = 0.0;
    public static double BL_MOTOR_POWER = 0.0;
    public static double BR_MOTOR_POWER = 0.0;

    private final Robot robot = Robot.getInstance();
    private TelemetryEx telemetryEx;

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

        robot.FLswervo.set(FL_SERVO_POS);
        robot.FRswervo.set(FR_SERVO_POS);
        robot.BLswervo.set(BL_SERVO_POS);
        robot.BRswervo.set(BR_SERVO_POS);

        telemetryEx.addData("FL Swervo Abs Pos", robot.FLswervo.getAbsoluteEncoder().getCurrentPosition());
        telemetryEx.addData("FR Swervo Abs Pos", robot.FRswervo.getAbsoluteEncoder().getCurrentPosition());
        telemetryEx.addData("BL Swervo Abs Pos", robot.BLswervo.getAbsoluteEncoder().getCurrentPosition());
        telemetryEx.addData("BR Swervo Abs Pos", robot.BRswervo.getAbsoluteEncoder().getCurrentPosition());
        
        // DO NOT REMOVE: Runs command scheduler, updates telemetry, and clears bulk cache
        robot.updateLoop(telemetryEx);
    }
}
