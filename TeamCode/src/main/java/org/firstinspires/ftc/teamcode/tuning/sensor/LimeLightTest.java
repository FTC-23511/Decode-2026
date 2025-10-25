package org.firstinspires.ftc.teamcode.tuning.sensor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.util.MathUtils;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp
public class LimeLightTest extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;

    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();

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

        telemetry.setMsTransmissionInterval(11);

        // pipeline 1 = AprilTag model
        robot.limelight.pipelineSwitch(0);

        robot.limelight.start();
    }

    @Override
    public void initialize_loop() {
        LLResult result = robot.limelight.getLatestResult();
        double heading = Math.toDegrees(robot.drive.getPose().getHeading() + MathUtils.normalizeRadians(robot.turretEncoder.getCurrentPosition(), false));

        robot.limelight.updateRobotOrientation(heading);

        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                int id = fiducial.getFiducialId();

                // Blue ID = 20
                // Obelisk PPG ID = 21
                // Obelisk PGP ID = 22
                // Obelisk PPG ID = 23
                // Red ID = 24

                if ((Constants.ALLIANCE_COLOR.equals(Constants.AllianceColor.BLUE) && id == 20)
                        || (Constants.ALLIANCE_COLOR.equals(Constants.AllianceColor.RED) && id == 24)) {

                    Pose3D botpose = result.getBotpose();
                    if (botpose != null) {
                        double x = botpose.getPosition().x;
                        double y = botpose.getPosition().y;
                        telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
                    } else {
                        telemetry.addData("MT1 Location", (Object) null);
                    }

                    Pose3D botpose_mt2 = result.getBotpose_MT2();
                    if (botpose_mt2 != null) {
                        double x = botpose_mt2.getPosition().x;
                        double y = botpose_mt2.getPosition().y;
                        telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                    } else {
                        telemetry.addData("MT2 Location", (Object) null);
                    }

                    telemetryData.addData("txPixels", fiducial.getTargetXPixels());
                    telemetryData.addData("tyPixels", fiducial.getTargetYPixels());
                    telemetryData.addData("txDegrees", fiducial.getTargetXDegrees());
                    telemetryData.addData("tyDegrees", fiducial.getTargetYDegrees());
                }
                else if (id == 21) {
                    telemetry.addData("Obelisk location:", "GPP");
                }
                else if (id == 22) {
                    telemetry.addData("Obelisk location:", "PGP");
                }
                else if (id == 23) {
                    telemetry.addData("Obelisk location:", "PPG");
                }
            }
        }

        telemetryData.addData("Heading", heading);
    }

    @Override
    public void run() {
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            robot.initHasMovement();
            timer = new ElapsedTime();
        }

        LLResult result = robot.limelight.getLatestResult();
        double heading = robot.drive.getPose().getHeading();

        robot.limelight.updateRobotOrientation(heading);

        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                int id = fiducial.getFiducialId();

                // Blue ID = 20
                // Obelisk PPG ID = 21
                // Obelisk PGP ID = 22
                // Obelisk PPG ID = 23
                // Red ID = 24

                if ((Constants.ALLIANCE_COLOR.equals(Constants.AllianceColor.BLUE) && id == 20)
                 || (Constants.ALLIANCE_COLOR.equals(Constants.AllianceColor.RED) && id == 24)) {
                    Pose3D botpose_mt2 = result.getBotpose_MT2();
                    if (botpose_mt2 != null) {
                        double x = botpose_mt2.getPosition().x;
                        double y = botpose_mt2.getPosition().y;
                        telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                    }
                }
                else if (id == 21) {
                    telemetry.addData("Obelisk location:", "GPP");
                }
                else if (id == 22) {
                    telemetry.addData("Obelisk location:", "PPG");
                }
                else if (id == 23) {
                    telemetry.addData("Obelisk location:", "PPG");
                }
            }
        }

        telemetryData.addData("Loop Time", timer.milliseconds());
        telemetryData.addData("Heading", heading);
        timer.reset();

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        super.run();
        telemetryData.update();
    }

    @Override
    public void end() {
//        Constants.END_POSE = robot.drive.getPose();
    }
}