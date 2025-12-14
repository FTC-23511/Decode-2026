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

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

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

        driver = new GamepadEx(gamepad1);

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
        if (gamepad1.cross) {
            Intake.distanceState = Intake.DistanceState.FOV_15;
        } else if (gamepad1.circle) {
            Intake.distanceState = Intake.DistanceState.FOV_20;
        } else if (gamepad1.triangle) {
            Intake.distanceState = Intake.DistanceState.FOV_27;
        }

//        telemetryData.addData("Front Threshold", FRONT_DISTANCE_THRESHOLD);
//        telemetryData.addData("Back Threshold", BACK_DISTANCE_THRESHOLD);
        telemetryData.addData("Distance Threshold", DISTANCE_THRESHOLD);
        telemetryData.addData("Distance Time", DISTANCE_TIME);
        telemetryData.addData("Actual Distance", robot.intake.getDistance());
        telemetryData.addData("Raw Voltage", robot.distanceSensor.getVoltage());
        telemetryData.addData("transferFull", robot.intake.transferFull());
//        telemetryData.addData("withinDistance", robot.intake.withinDistance);
//        telemetryData.addData("distanceTimer (ms)", robot.intake.distanceTimer.milliseconds());

//        telemetryData.addData("Threshold Met", robot.frontDistanceSensor.isActive());
//        telemetryData.addData("Back Threshold Met", robot.backDistanceSensor.isActive());

        robot.updateLoop(telemetryData);
    }
}