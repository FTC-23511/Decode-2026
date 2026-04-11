package org.firstinspires.ftc.teamcode.tuning.sensor;

import static org.firstinspires.ftc.teamcode.globals.Constants.TESTING_OP_MODE;

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
import com.seattlesolvers.solverslib.util.TelemetryEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp(name = "LimelightTest", group = "Sensor")
public class LimelightTest extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;

    public static double ROBOT_HEADING = 0;

    public static boolean USE_PINPOINT_HEADING = false;

    TelemetryEx telemetryEx = new TelemetryEx(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();

    @Override
    public void initialize() {
        // Must have for all opModes
        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;
        TESTING_OP_MODE = true;

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
    public void run() {
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            robot.initHasMovement();
            timer = new ElapsedTime();
        }

        LLResult result = robot.limelight.getLatestResult();
        double heading =  MathUtils.normalizeDegrees(
                robot.drive.getPose().getRotation().getDegrees()
                        + Math.toDegrees(robot.turret.getPosition()) + 90.0, true);

        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                int id = fiducial.getFiducialId();

                // Blue ID = 20
                // Obelisk PPG ID = 21
                // Obelisk PGP ID = 22
                // Obelisk PPG ID = 23
                // Red ID = 24

                telemetryEx.addData("Tag ID", id);

                if ((Constants.ALLIANCE_COLOR.equals(Constants.AllianceColor.BLUE) && id == 20)
                        || (Constants.ALLIANCE_COLOR.equals(Constants.AllianceColor.RED) && id == 24)) {

                    if (USE_PINPOINT_HEADING) {
                        robot.limelight.updateRobotOrientation(heading);
                    } else {
                        robot.limelight.updateRobotOrientation(ROBOT_HEADING);
                    }

                    Pose3D botpose = result.getBotpose();
                    if (botpose != null) {
                        double x = botpose.getPosition().x;
                        double y = botpose.getPosition().y;
                        double z = botpose.getPosition().z;

                        x = DistanceUnit.INCH.fromMeters(x);
                        y = DistanceUnit.INCH.fromMeters(y);
                        z = DistanceUnit.INCH.fromMeters(z);

                        telemetry.addData("MT1 Location", "(" + x + ", " + y + ", " + z + ")");
                    } else {
                        telemetry.addData("MT1 Location", "null");
                    }

                    Pose3D botpose_mt2 = result.getBotpose_MT2();
                    if (botpose_mt2 != null) {
                        double x = botpose_mt2.getPosition().x;
                        double y = botpose_mt2.getPosition().y;
                        double z = botpose_mt2.getPosition().z;

                        x = DistanceUnit.INCH.fromMeters(x);
                        y = DistanceUnit.INCH.fromMeters(y);
                        z = DistanceUnit.INCH.fromMeters(z);

                        telemetry.addData("MT2 Location", "(" + x + ", " + y + ", " + z + ")");
                    } else {
                        telemetry.addData("MT2 Location", "null");
                    }

                    telemetryEx.addData("txPixels", fiducial.getTargetXPixels());
                    telemetryEx.addData("tyPixels", fiducial.getTargetYPixels());
                    telemetryEx.addData("txDegrees", fiducial.getTargetXDegrees());
                    telemetryEx.addData("tyDegrees", fiducial.getTargetYDegrees());

                    telemetryEx.addData("robotPoseTargetSpace", fiducial.getRobotPoseTargetSpace()); // Robot pose relative it the AprilTag Coordinate System (Most Useful)
                    telemetryEx.addData("cameraPoseTargetSpace", fiducial.getCameraPoseTargetSpace()); // Camera pose relative to the AprilTag (useful)
                    telemetryEx.addData("robotPoseFieldSpace", fiducial.getRobotPoseFieldSpace()); // Robot pose in the field coordinate system based on this tag alone (useful)
                    telemetryEx.addData("targetPoseCameraSpace", fiducial.getTargetPoseCameraSpace()); // AprilTag pose in the camera's coordinate system (not very useful)
                    telemetryEx.addData("targetPoseRobotSpace", fiducial.getTargetPoseRobotSpace()); // AprilTag pose in the robot's coordinate system (not very useful)
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

        telemetryEx.addData("Loop Time", timer.milliseconds());
        telemetryEx.addData("Heading", heading);
        timer.reset();

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        robot.updateLoop(telemetryEx);
    }

    @Override
    public void end() {
//        Constants.END_POSE = robot.drive.getPose();
    }
}