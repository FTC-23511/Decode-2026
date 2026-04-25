package org.firstinspires.ftc.teamcode.globals;

import com.outoftheboxrobotics.photoncore.PhotonCore;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.util.TelemetryEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Camera;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;

import dev.nullftc.profiler.Profiler;
import dev.nullftc.profiler.entry.BasicProfilerEntryFactory;
import dev.nullftc.profiler.exporter.CSVProfilerExporter;


public class Robot extends com.seattlesolvers.solverslib.command.Robot {
    private static final Robot instance = new Robot();
    public static Robot getInstance() {
        return instance;
    }

    public Profiler profiler;
    public File profilerFile;
    public File logCatFile;

    public VoltageSensor voltageSensor;
    private double cachedVoltage;
    private ElapsedTime voltageTimer;

    public boolean readyToLaunch = true;

    public MotorEx FRmotor;
    public MotorEx FLmotor;
    public MotorEx BLmotor;
    public MotorEx BRmotor;

    public MotorGroup intakeMotor;

    public MotorGroup launchMotors;
    public Motor.Encoder launchEncoder;

    public MotorEx transferMotor;

    public CRServoEx FRswervo;
    public CRServoEx FLswervo;
    public CRServoEx BLswervo;
    public CRServoEx BRswervo;

    public ServoExGroup turretServos;
    public Motor.Encoder turretEncoder;
    public AbsoluteAnalogEncoder analogTurretEncoder;

    public ServoEx hoodServo;
    public ServoEx rampServo;

//    public Limelight3A limelight;

    public OctoQuadFWv3 octoQuad;
    public OctoQuadFWv3.LocalizerDataBlock localizer = new OctoQuadFWv3.LocalizerDataBlock();
//    public GoBildaPinpointDriver pinpoint;
//    public IMU imu;

    public Drive drive;
    public Intake intake;
    public Launcher launcher;
    public Turret turret;
    public Camera camera;

    public AnalogInput distanceSensor;
    private MathFunctions.VirtualGoalSolver.ShotSolution shotSolution;

    public void init(HardwareMap hwMap) {
        File profilerFolder = new File(AppUtil.FIRST_FOLDER, "logs");
        File logcatFolder = new File(AppUtil.FIRST_FOLDER, "logcat");
        if (!profilerFolder.exists()) profilerFolder.mkdirs();
        if (!logcatFolder.exists()) logcatFolder.mkdirs();

        long timestamp = System.currentTimeMillis();
        profilerFile = new File(profilerFolder, "profiler-" + timestamp + ".csv");
        logCatFile = new File(logcatFolder, "logcat_" + timestamp + ".txt");

        profiler = Profiler.builder()
                .factory(new BasicProfilerEntryFactory())
                .exporter(new CSVProfilerExporter(profilerFile))
                .debugLog(false) // Log EVERYTHING
                .build();

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

        // Hardware
        voltageSensor = hwMap.voltageSensor.iterator().next();

        FRmotor = new MotorEx(hwMap, "FR").setCachingTolerance(0.01);
        FLmotor = new MotorEx(hwMap, "FL").setCachingTolerance(0.01);
        BLmotor = new MotorEx(hwMap, "BL").setCachingTolerance(0.01);
        BRmotor = new MotorEx(hwMap, "BR").setCachingTolerance(0.01);

        FRmotor.setRunMode(Motor.RunMode.RawPower);
        FLmotor.setRunMode(Motor.RunMode.RawPower);
        BLmotor.setRunMode(Motor.RunMode.RawPower);
        BRmotor.setRunMode(Motor.RunMode.RawPower);

        launchMotors = new MotorGroup(
                new MotorEx(hwMap, "leftLaunchMotor")
                        .setCachingTolerance(0.01)
                        .setInverted(true),
                new MotorEx(hwMap, "rightLaunchMotor")
                        .setCachingTolerance(0.01)
        );

        launchMotors.setRunMode(Motor.RunMode.RawPower);
        launchMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        launchEncoder = new Motor(hwMap, "FL").encoder
                .setDirection(Motor.Direction.FORWARD)
                .setVelocityLimit(5000);

        transferMotor = new MotorEx(hwMap, "transferMotor")
                .setCachingTolerance(0.01);
        transferMotor.setRunMode(Motor.RunMode.RawPower)
                .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
                .setInverted(false);

        intakeMotor = new MotorGroup(
                new MotorEx(hwMap, "frontIntakeMotor")
                        .setCachingTolerance(0.01)
                        .setCurrentAlert(INTAKE_CURRENT_THRESHOLD, CurrentUnit.MILLIAMPS)
                        .setRunMode(Motor.RunMode.RawPower)
                        .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
//                ,transferMotor
        );

        FRswervo = new CRServoEx(hwMap, "FR", new AbsoluteAnalogEncoder(hwMap, "FR")
                .zero(FR_ENCODER_OFFSET), CRServoEx.RunMode.RawPower)
                .setCachingTolerance(0.02);
        FLswervo = new CRServoEx(hwMap, "FL", new AbsoluteAnalogEncoder(hwMap, "FL")
                .zero(FL_ENCODER_OFFSET), CRServoEx.RunMode.RawPower)
                .setCachingTolerance(0.02);
        BLswervo = new CRServoEx(hwMap, "BL", new AbsoluteAnalogEncoder(hwMap, "BL")
                .zero(BL_ENCODER_OFFSET), CRServoEx.RunMode.RawPower)
                .setCachingTolerance(0.02);
        BRswervo = new CRServoEx(hwMap, "BR", new AbsoluteAnalogEncoder(hwMap, "BR")
                .zero(BR_ENCODER_OFFSET), CRServoEx.RunMode.RawPower)
                .setCachingTolerance(0.02);

        turretServos = new ServoExGroup(
                new ServoEx(hwMap, "leftTurretServo")
                        .setCachingTolerance(0.001),
                new ServoEx(hwMap, "rightTurretServo")
                        .setCachingTolerance(0.001)

        ).setInverted(true);

        turretServos.set(0.5);

        analogTurretEncoder = new AbsoluteAnalogEncoder(hwMap, "turretEncoder")
                .zero(TURRET_ENCODER_OFFSET)
                .setReversed(true);

        turretEncoder = new Motor(hwMap, "BL").encoder
                .setDirection(Motor.Direction.FORWARD)
                .overrideResetPos((int) TURRET_SYNC_OFFSET);

        hoodServo = new ServoEx(hwMap, "hoodServo").setCachingTolerance(0.001)
                .setInverted(true);
        rampServo = new ServoEx(hwMap, "rampServo").setCachingTolerance(0.001)
                .setInverted(false);
//        stopperServo = new ServoEx(hwMap, "stopperServo").setCachingTolerance(0.001)
//                .setInverted(true);

        octoQuad = hwMap.get(OctoQuadFWv3.class, "octoquad")
                .setAllLocalizerParameters(0, 1, 13.26291192f, 13.26291192f, -152.50f, 76.32f, 1.0275f, 15)
                .setSingleEncoderDirection(0, OctoQuadFWv3.EncoderDirection.FORWARD)
                .setSingleEncoderDirection(1, OctoQuadFWv3.EncoderDirection.REVERSE)
                .setI2cRecoveryMode(OctoQuadFWv3.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR)
                .resetLocalizerAndCalibrateIMU();

//        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint")
//                .setOffsets(-76.32, 152.50, DistanceUnit.MM)
//                .setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
//                .setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED)
//                .setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.LOCAL_TEST)
//                .setBulkReadScope(GoBildaPinpointDriver.Register.X_POSITION, GoBildaPinpointDriver.Register.Y_POSITION, GoBildaPinpointDriver.Register.H_ORIENTATION, GoBildaPinpointDriver.Register.X_VELOCITY, GoBildaPinpointDriver.Register.Y_VELOCITY, GoBildaPinpointDriver.Register.H_VELOCITY)
//                .setPosition(Pose2d.convertToPose2D(END_POSE, DISTANCE_UNIT, ANGLE_UNIT));

        distanceSensor = hwMap.get(AnalogInput.class, "distanceSensor");

//        limelight = hwMap.get(Limelight3A.class, "limelight");
//        limelight.setPollRateHz(250);
//        limelight.pipelineSwitch(4);
//        limelight.start();

        // Subsystems
        drive = new Drive();
        intake = new Intake();
        launcher = new Launcher();
        turret = new Turret();
        camera = new Camera();

        // Robot/CommandScheduler configurations
//        setBulkReading(hwMap, LynxModule.BulkCachingMode.MANUAL);

//        for (LynxModule hub : hwMap.getAll(LynxModule.class)) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//            if (hub.isParent() && LynxConstants.isEmbeddedSerialNumber(hub.getSerialNumber())) {
//                controlHub = hub;
//            } else if (!hub.isParent() && hub.getRevProductNumber() == (EXPANSION_HUB_PRODUCT_NUMBER)) {
//                expansionHub = hub;
//            } else if (!hub.isParent() && hub.getRevProductNumber() == (SERVO_HUB_PRODUCT_NUMBER)) {
//                servoHub = hub;
//            }
//        }

//        register(drive, intake, launcher, turret); // not needed bc SubsystemBase registers it anyways 💀

        if (OP_MODE_TYPE.equals(OpModeType.AUTO)) {
            initHasMovement();
        }
    }

    public void initHasMovement() {
        drive.init();
        intake.init();
        launcher.init();
        turret.init();
        camera.initHasMovement();
    }
    
    public double getVoltage() {
//        return 12;
        // this is chopped for loop times
        if (voltageTimer == null) {
            cachedVoltage = voltageSensor.getVoltage();
            voltageTimer = new ElapsedTime();
            voltageTimer.reset();
        } else if (voltageTimer.milliseconds() > (1.0 / VOLTAGE_SENSOR_POLLING_RATE) * 1000) {
            cachedVoltage = voltageSensor.getVoltage();
        }
        if (Double.isNaN(cachedVoltage) || cachedVoltage <= 0) {
            cachedVoltage = 12;
        }
        return cachedVoltage;
    }

    public void exportProfiler(File profilerFile, File logCatFile) {
        RobotLog.i("Starting async profiler and logcat export to: " + profilerFile.getAbsolutePath());

        Thread exportThread = new Thread(() -> {
            try {
                profiler.export();
                profiler.shutdown();
            } catch (Exception e) {
                RobotLog.e("An error occurred", e.toString());
                RobotLog.e(e.toString(), Arrays.toString(e.getStackTrace()));
            }

            try {
                Process process = Runtime.getRuntime().exec("logcat -d -f " + logCatFile.getAbsolutePath());

                int exitCode = process.waitFor();

                if (exitCode == 0) {
                    RobotLog.i("Logcat export successful to " + logCatFile.getAbsolutePath());
                } else {
                    RobotLog.w("Logcat export failed with exit code: " + exitCode);
                }
            } catch (IOException | InterruptedException e) {
                RobotLog.i("Logcat export Error", e.getMessage());
            }
        });

        exportThread.setDaemon(true);
        exportThread.start();
    }

    /* Not needed now that we have pinpoint
    public void initializeImu(HardwareMap hardwareMap) {
        // IMU orientation
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }
     */

    public MathFunctions.VirtualGoalSolver.ShotSolution getShotSolution() {
        if (shotSolution == null) {
            // 1. Get raw data
            Pose2d robotPose = drive.getPose();
            Rotation2d robotRotation = robotPose.getRotation();

            // Sensor velocity (Robot-Centric from Pinpoint/Odometry)
            ChassisSpeeds robotVelRobot = drive.getVelocity();

            // Driver intent (Robot-Centric from joysticks)
            ChassisSpeeds targetVelRobot = drive.swerve.getTargetVelocity();

            // 2. Predict Linear Velocity (Robot-Centric)
            double predictedVxRobot = robotVelRobot.vxMetersPerSecond +
                    (targetVelRobot.vxMetersPerSecond - robotVelRobot.vxMetersPerSecond) * DRIVE_VEL_PREDICT_ALPHA;

            double predictedVyRobot = robotVelRobot.vyMetersPerSecond +
                    (targetVelRobot.vyMetersPerSecond - robotVelRobot.vyMetersPerSecond) * DRIVE_VEL_PREDICT_ALPHA;

            // 3. Predict Angular Velocity (Radians/sec)
            double predictedOmegaRadPerSec = robotVelRobot.omegaRadiansPerSecond +
                    (targetVelRobot.omegaRadiansPerSecond - robotVelRobot.omegaRadiansPerSecond) * DRIVE_VEL_PREDICT_ALPHA;

            // Convert predicted robot-centric velocity to field-centric for the solver
            ChassisSpeeds predictedVelField = ChassisSpeeds.toFieldRelativeSpeeds(
                    new ChassisSpeeds(predictedVxRobot, predictedVyRobot, predictedOmegaRadPerSec),
                    robotRotation
            );

            Vector2d predictedLinearVelIps = new Vector2d(predictedVelField.vxMetersPerSecond, predictedVelField.vyMetersPerSecond);

            // 4. Solve
            shotSolution = MathFunctions.VirtualGoalSolver.solve(
                    robotPose,
                    predictedLinearVelIps,
                    predictedOmegaRadPerSec,
                    turret.adjustedGoalPose(),
                    Launcher.timeOfFlightLUT
            );
        }

        return shotSolution;
    }

    public void updateLoop(TelemetryEx telemetryEx) {
        super.run();

        shotSolution = null;

        if (telemetryEx != null) {
            telemetryEx.update();
        }

        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();
    }

    public void initializeLoop(Gamepad gamepad1, TelemetryEx telemetryEx) {
        if (gamepad1.right_stick_button) {
            TURRET_SYNCED = false;
            turret.resetTurretEncoder();
            octoQuad.resetLocalizerAndCalibrateIMU();
//            robot.pinpoint.recalibrateIMU();
        }
        if (OP_MODE_TYPE.equals(OpModeType.TELEOP)) {
            Launcher.DISTANCE_OFFSET = 0;
            telemetryEx.addData("END_POSE", END_POSE);
        }

        telemetryEx.addData("Localizer Status", octoQuad.getLocalizerStatus());
        telemetryEx.addData("TURRET_SYNCED", TURRET_SYNCED);
        telemetryEx.addData("Alliance Color", ALLIANCE_COLOR);
        telemetryEx.update();

        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();
    }
}