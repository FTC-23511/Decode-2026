package org.firstinspires.ftc.teamcode.tuning.sensor;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp(name = "DistanceSensorTest", group = "Sensor")
public class DistanceSensorTest extends CommandOpMode {

    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();
    private GamepadEx driver;

    @Override
    public void initialize() {
        // Must have for all opModes
        Constants.OP_MODE_TYPE = OpModeType.TELEOP;

        // Resets the command scheduler
        super.reset();

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);

        driver = new GamepadEx(gamepad1);

        robot.launcher.setRamp(false);

        driver.getGamepadButton(GamepadKeys.Button.TRIANGLE).toggleWhenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intake.setIntake(Intake.MotorState.FORWARD))
                ),
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intake.setIntake(Intake.MotorState.STOP))
                )
        );
    }

    @Override
    public void run() {
//        telemetryData.addData("Front Threshold", FRONT_DISTANCE_THRESHOLD);
//        telemetryData.addData("Back Threshold", BACK_DISTANCE_THRESHOLD);
        telemetryData.addData("Distance Threshold", INTAKE_DISTANCE_THRESHOLD);
        telemetryData.addData("Actual Distance", robot.intake.getDistance());
        telemetryData.addData("Distance Timer", robot.intake.distanceTimer.milliseconds());
        telemetryData.addData("Distance Timer", robot.intake.distanceTimer.milliseconds());
        telemetryData.addData("transferFull", robot.intake.transferFull());
//        telemetryData.addData("withinDistance", robot.intake.withinDistance);
//        telemetryData.addData("distanceTimer (ms)", robot.intake.distanceTimer.milliseconds());

//        telemetryData.addData("Threshold Met", robot.frontDistanceSensor.isActive());
//        telemetryData.addData("Back Threshold Met", robot.backDistanceSensor.isActive());

        robot.updateLoop(telemetryData);
    }
}