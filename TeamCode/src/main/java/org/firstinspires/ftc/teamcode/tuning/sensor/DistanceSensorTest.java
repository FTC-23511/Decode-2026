package org.firstinspires.ftc.teamcode.tuning.sensor;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Disabled
@Deprecated
@Config
@TeleOp(name = "DistanceSensorTest", group = "Sensor")
public class DistanceSensorTest extends CommandOpMode {
    public GamepadEx driver;


    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();

    @Override
    public void initialize() {
        // Must have for all opModes
        Constants.OP_MODE_TYPE = OpModeType.TELEOP;

        // Resets the command scheduler
        super.reset();

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);
    }

    @Override
    public void initialize_loop() {
        update();
    }

    @Override
    public void run() {
        update();
    }

    public void update() {
        telemetryData.addData("Front Threshold", FRONT_DISTANCE_THRESHOLD);
        telemetryData.addData("Back Threshold", BACK_DISTANCE_THRESHOLD);

        telemetryData.addData("Front Threshold Met", robot.frontDistanceSensor.isActive());
        telemetryData.addData("Back Threshold Met", robot.backDistanceSensor.isActive());

        telemetryData.update();
    }
}