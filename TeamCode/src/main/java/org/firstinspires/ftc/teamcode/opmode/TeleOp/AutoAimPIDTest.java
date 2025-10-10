package org.firstinspires.ftc.teamcode.opmode.TeleOp;
import static org.firstinspires.ftc.teamcode.globals.Constants.HOOD_SET_POSITION_TUNER;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@TeleOp(name="AprilTag Test Limelight", group="Vision")
public class AutoAimPIDTest extends OpMode {

    private Limelight3A limelight;
    private final Robot robot = Robot.getInstance();
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;

    TelemetryData telemetryData = new TelemetryData(telemetry);


    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0);
        // Must have for all opModes
        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;

        // Resets the command scheduler
        CommandScheduler.getInstance().reset();
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
    public void start() {
        limelight.start();

    }

    @Override
    public void loop() {
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                int id = fiducial.getFiducialId();


                double txDegrees = fiducial.getTargetXDegrees();
                double tyDegrees = fiducial.getTargetYDegrees();
                double tyPixels = fiducial.getTargetYPixels();
                double txPixels = fiducial.getTargetXPixels();

                // Blue ID = 20
                // Obelisk PPG ID = 21
                // Obelisk PGP ID = 22
                // Obelisk PPG ID = 23
                // Red ID = 24

                if ((Constants.ALLIANCE_COLOR.equals(Constants.AllianceColor.BLUE) && id == 20)
                        || (Constants.ALLIANCE_COLOR.equals(Constants.AllianceColor.RED) && id == 24)) {

                    if (txDegrees > 5) {
                        robot.turretServo.set(0.1);
                    } else if (txDegrees < -5) {
                        robot.turretServo.set(-0.1);
                    } else {
                        robot.turretServo.set(0);
                    }


                    if ((tyDegrees > 5) || (tyDegrees < -5)) {
                        robot.hoodServo.set(0.5+(tyDegrees*HOOD_SET_POSITION_TUNER));
                    }


                    telemetryData.addData("txPixels", txPixels);
                    telemetryData.addData("tyPixels", tyPixels);
                    telemetryData.addData("txDegrees", txDegrees);
                    telemetryData.addData("tyDegrees", tyDegrees);
                }
            }
        }
    }
}

