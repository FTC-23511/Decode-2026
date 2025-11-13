package org.firstinspires.ftc.teamcode.tuning.sensor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

//@Config
@TeleOp(name = "LimelightAngle", group = "Sensor")
public class LimelightAngle extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;

    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();
    Pose2d lastKnownPose = new Pose2d();


    @Override
    public void initialize() {
        // Must have for all opModes
        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;

        // Resets the command scheduler
        super.reset();

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        // Driver controls
        // TODO: add controls here

        // Operator controls
        // TODO: add controls here
    }

    @Override
    public void run() {
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            robot.initHasMovement();
            timer = new ElapsedTime();
        }

        // TODO: Add robot drive movement here


        if (robot.turret.getLimelightPose() != null) {
            lastKnownPose = robot.turret.getLimelightPose();

        }
        robot.turret.updateLLResult(5);


        telemetryData.addData("loop time", timer.milliseconds());
        timer.reset();
        telemetryData.addData("Angle (Radians)", robot.turret.tyOffset(lastKnownPose));
        telemetryData.addData("Angle (Degrees)", Math.toDegrees(robot.turret.tyOffset(lastKnownPose)));
        telemetryData.addData("bot pose", lastKnownPose);


        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        robot.updateLoop(telemetryData);
    }

    @Override
    public void end() {
//        Constants.END_POSE = robot.drive.getPose();
    }
}