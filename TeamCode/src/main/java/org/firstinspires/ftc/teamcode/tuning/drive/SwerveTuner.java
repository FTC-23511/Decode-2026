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
@TeleOp(name = "SwerveTuner", group = "Tuning")
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
        // Apply static variables directly to hardware components (bypassing kinematics)
        robot.FLmotor.set(FL_MOTOR_POWER);
        robot.FRmotor.set(FR_MOTOR_POWER);
        robot.BLmotor.set(BL_MOTOR_POWER);
        robot.BRmotor.set(BR_MOTOR_POWER);

        robot.FLswervo.set(FL_SERVO_POS);
        robot.FRswervo.set(FR_SERVO_POS);
        robot.BLswervo.set(BL_SERVO_POS);
        robot.BRswervo.set(BR_SERVO_POS);

        // Output tuning values to telemetry
        telemetryEx.addData("FL Servo Pos Input", FL_SERVO_POS);
        telemetryEx.addData("FR Servo Pos Input", FR_SERVO_POS);
        telemetryEx.addData("BL Servo Pos Input", BL_SERVO_POS);
        telemetryEx.addData("BR Servo Pos Input", BR_SERVO_POS);

        telemetryEx.addData("FL Motor Power Input", FL_MOTOR_POWER);
        telemetryEx.addData("FR Motor Power Input", FR_MOTOR_POWER);
        telemetryEx.addData("BL Motor Power Input", BL_MOTOR_POWER);
        telemetryEx.addData("BR Motor Power Input", BR_MOTOR_POWER);

        // Drive motor encoders (positions and velocities)
        telemetryEx.addData("FL Drive Pos", robot.FLmotor.getCurrentPosition());
        telemetryEx.addData("FL Drive Vel", robot.FLmotor.getVelocity());
        telemetryEx.addData("FR Drive Pos", robot.FRmotor.getCurrentPosition());
        telemetryEx.addData("FR Drive Vel", robot.FRmotor.getVelocity());
        telemetryEx.addData("BL Drive Pos", robot.BLmotor.getCurrentPosition());
        telemetryEx.addData("BL Drive Vel", robot.BLmotor.getVelocity());
        telemetryEx.addData("BR Drive Pos", robot.BRmotor.getCurrentPosition());
        telemetryEx.addData("BR Drive Vel", robot.BRmotor.getVelocity());

        // Swerve module rotation absolute encoders (for finding zero-offsets)
        telemetryEx.addData("FL Swervo Abs Pos", robot.FLswervo.getAbsoluteEncoder().getCurrentPosition());
        telemetryEx.addData("FR Swervo Abs Pos", robot.FRswervo.getAbsoluteEncoder().getCurrentPosition());
        telemetryEx.addData("BL Swervo Abs Pos", robot.BLswervo.getAbsoluteEncoder().getCurrentPosition());
        telemetryEx.addData("BR Swervo Abs Pos", robot.BRswervo.getAbsoluteEncoder().getCurrentPosition());

        telemetryEx.addData("FL Swervo Abs Volts", robot.FLswervo.getAbsoluteEncoder().getVoltage());
        telemetryEx.addData("FR Swervo Abs Volts", robot.FRswervo.getAbsoluteEncoder().getVoltage());
        telemetryEx.addData("BL Swervo Abs Volts", robot.BLswervo.getAbsoluteEncoder().getVoltage());
        telemetryEx.addData("BR Swervo Abs Volts", robot.BRswervo.getAbsoluteEncoder().getVoltage());

        // DO NOT REMOVE: Runs command scheduler, updates telemetry, and clears bulk cache
        robot.updateLoop(telemetryEx);
    }
}
