package org.firstinspires.ftc.teamcode.opmode.TeleOp;
import static org.firstinspires.ftc.teamcode.globals.Constants.BLUE_TARGET_X;
import static org.firstinspires.ftc.teamcode.globals.Constants.BLUE_TARGET_Y;
import static org.firstinspires.ftc.teamcode.globals.Constants.EXIT_VELO;
import static org.firstinspires.ftc.teamcode.globals.Constants.G;
import static org.firstinspires.ftc.teamcode.globals.Constants.HOOD_SET_POSITION_TUNER;
import static org.firstinspires.ftc.teamcode.globals.Constants.H_TARGET;
import static org.firstinspires.ftc.teamcode.globals.Constants.RED_TARGET_X;
import static org.firstinspires.ftc.teamcode.globals.Constants.RED_TARGET_Y;
import static org.firstinspires.ftc.teamcode.globals.Constants.SHOOTER_DX;
import static org.firstinspires.ftc.teamcode.globals.Constants.SHOOTER_DY;
import static org.firstinspires.ftc.teamcode.globals.Constants.SHOOTER_RELEASE_HEIGHT;

import com.acmerobotics.dashboard.config.Config;
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
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Transform2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp(name="AprilTag Test Limelight", group="Vision")
public class AutoAimPIDTest extends OpMode {

    private Limelight3A limelight;
    private final Robot robot = Robot.getInstance();
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;
    public static double TUNER_POWER = 0.2;



    TelemetryData telemetryData = new TelemetryData(telemetry);


    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
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

        Pose2d robotPose = robot.drive.getPose();
        Transform2d shooterOffset = new Transform2d(new Translation2d(SHOOTER_DX, SHOOTER_DY), new Rotation2d(0.0));
        Pose2d shooterPose = robotPose.plus(shooterOffset);

        double targetX, targetY;
        if (Constants.ALLIANCE_COLOR == Constants.AllianceColor.BLUE) {
            targetX = BLUE_TARGET_X;
            targetY = BLUE_TARGET_Y;
        } else {
            targetX = RED_TARGET_X;
            targetY = RED_TARGET_Y;
        }

        double dx = targetX - shooterPose.getX();
        double dy = targetY - shooterPose.getY();
        double d  = Math.hypot(dx, dy);
        double dH = H_TARGET - SHOOTER_RELEASE_HEIGHT;

        double desiredHeading = Math.atan2(dy, dx);
        double robotHeading   = robotPose.getRotation().getRadians();
        double delta          = desiredHeading - robotHeading;
        double turretYawRad   = Math.atan2(Math.sin(delta), Math.cos(delta));

        double v2 = EXIT_VELO * EXIT_VELO;
        double discriminant = v2*v2 - G * (G*d*d + 2.0*dH*v2);

        Double hoodLowRad = null, hoodHighRad = null;
        if (discriminant >= 0.0 && d > 1e-6) {
            double root = Math.sqrt(discriminant);
            double tanHigh = (v2 + root) / (G*d);
            double tanLow  = (v2 - root) / (G*d);
            hoodHighRad = Math.atan(tanHigh);
            hoodLowRad  = Math.atan(tanLow);
        }




        LLStatus status = limelight.getStatus();



        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                int id = fiducial.getFiducialId();


                double txDegrees = fiducial.getTargetYDegrees();
                double tyDegrees = fiducial.getTargetXDegrees();
                double tyPixels = fiducial.getTargetXPixels();
                double txPixels = fiducial.getTargetYPixels();



                // Blue ID = 20
                // Obelisk PPG ID = 21
                // Obelisk PGP ID = 22
                // Obelisk PPG ID = 23
                // Red ID = 24

                if ((Constants.ALLIANCE_COLOR.equals(Constants.AllianceColor.BLUE) && id == 20)
                        || (Constants.ALLIANCE_COLOR.equals(Constants.AllianceColor.RED) && id == 24)) {

                    if (txDegrees > 5) {
                        robot.rightTurretServo.set(TUNER_POWER);
                        robot.leftTurretServo.set(TUNER_POWER);
                        telemetryData.addData("Pos Tuner", TUNER_POWER);
                    } else if (txDegrees < -5) {
                        robot.rightTurretServo.set(-TUNER_POWER);
                        robot.leftTurretServo.set(-TUNER_POWER);
                        telemetryData.addData("Neg Tuner", TUNER_POWER);


                    } else {
                        robot.rightTurretServo.set(0);
                        robot.leftTurretServo.set(0);
                        telemetryData.addData("0 Tuner", TUNER_POWER);


                    }


                    if ((tyDegrees > 5) || (tyDegrees < -5)) {
                        robot.hoodServo.set(0.5+(tyDegrees*HOOD_SET_POSITION_TUNER));
                    }




                    telemetryData.addData("txPixels", txPixels);
                    telemetryData.addData("tyPixels", tyPixels);
                    telemetryData.addData("txDegrees", txDegrees);
                    telemetryData.addData("tyDegrees", tyDegrees);
                    telemetryData.addData("TurretYaw (rad)", turretYawRad);
                    if (hoodLowRad != null) {
                        telemetryData.addData("HoodLow (rad)", hoodLowRad);
                        telemetryData.addData("HoodHigh (rad)", hoodHighRad);
                    } else {
                        telemetryData.addData("Hood", "No Solution");
                    }
                    telemetryData.update();
                }
            }
        }
    }
}

