package org.firstinspires.ftc.teamcode.globals;

import com.outoftheboxrobotics.photoncore.PhotonCore;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
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

//    public LynxModule controlHub;
//    public LynxModule expansionHub;
//    public LynxModule servoHub;
//    public ArrayList<LynxModule> hubs;
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
    public Motor.Encoder transferEncoder;
    public ServoEx stopperServo;

    public CRServoEx FRswervo;
    public CRServoEx FLswervo;
    public CRServoEx BLswervo;
    public CRServoEx BRswervo;

    public ServoExGroup turretServos;
    public Motor.Encoder turretEncoder;
    public AbsoluteAnalogEncoder analogTurretEncoder;

    public ServoEx hoodServo;
    public ServoEx rampServo;

    public Limelight3A limelight;

    public GoBildaPinpointDriver pinpoint;
//    public IMU imu;

    public Drive drive;
    public Intake intake;
    public Launcher launcher;
    public Turret turret;
    public Camera camera;

//    public SensorDigitalDevice frontDistanceSensor;
//    public SensorDigitalDevice backDistanceSensor;
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

        intakeMotor = new MotorGroup(
                new MotorEx(hwMap, "frontIntakeMotor")
                        .setCachingTolerance(0.01)
                        .setCurrentAlert(INTAKE_CURRENT_THRESHOLD, CurrentUnit.MILLIAMPS)
                        .setRunMode(Motor.RunMode.RawPower)
                        .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        );

        launchMotors = new MotorGroup(
                new MotorEx(hwMap, "leftLaunchMotor")
                        .setCachingTolerance(0.01)
                        .setInverted(true),
                new MotorEx(hwMap, "rightLaunchMotor")
                        .setCachingTolerance(0.01)
        );

        launchMotors.setRunMode(Motor.RunMode.RawPower);
        launchMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        launchEncoder = new Motor(hwMap, "FL").encoder.
                setDirection(Motor.Direction.FORWARD);

        transferMotor = new MotorEx(hwMap, "transferMotor")
                .setCachingTolerance(0.01);
        transferMotor.setRunMode(Motor.RunMode.RawPower)
                .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
                .setInverted(true);

        transferEncoder = new Motor(hwMap, "BL").encoder
                .setDirection(Motor.Direction.FORWARD);

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

        RobotLog.aa("VERY First Voltage", String.valueOf(analogTurretEncoder.getVoltage()));

        turretEncoder = new Motor(hwMap, "BL").encoder
                .setDirection(Motor.Direction.FORWARD)
                .overrideResetPos((int) TURRET_SYNC_OFFSET);

        hoodServo = new ServoEx(hwMap, "hoodServo").setCachingTolerance(0.001)
                .setInverted(true);
        rampServo = new ServoEx(hwMap, "rampServo").setCachingTolerance(0.001)
                .setInverted(false);
        stopperServo = new ServoEx(hwMap, "stopperServo").setCachingTolerance(0.001)
                .setInverted(true);

        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint")
                .setOffsets(-76.32, 152.50, DistanceUnit.MM)
                .setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
                .setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED)
                .setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC)
                .resetPosAndIMU()
                .setPosition(Pose2d.convertToPose2D(END_POSE, DistanceUnit.INCH, AngleUnit.RADIANS))
                .setBulkReadScope(GoBildaPinpointDriver.Register.X_POSITION, GoBildaPinpointDriver.Register.Y_POSITION, GoBildaPinpointDriver.Register.H_ORIENTATION, GoBildaPinpointDriver.Register.X_VELOCITY, GoBildaPinpointDriver.Register.Y_VELOCITY, GoBildaPinpointDriver.Register.H_VELOCITY);

//        frontDistanceSensor = new SensorDigitalDevice(hwMap, "frontDistanceSensor", FRONT_DISTANCE_THRESHOLD);
//        backDistanceSensor = new SensorDigitalDevice(hwMap, "backDistanceSensor", BACK_DISTANCE_THRESHOLD);

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
        camera = new Camera(hwMap);

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
        RobotLog.i("Starting async profiler and Logcat export to: " + profilerFile.getAbsolutePath());

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

            // Sensor velocity (Already Field-Centric from Pinpoint/Odometry)
            ChassisSpeeds robotVelField = drive.getVelocity();

            // Driver intent (Robot-Centric from joysticks, needs conversion)
            ChassisSpeeds targetVelRobot = drive.swerve.getTargetVelocity();
            ChassisSpeeds targetVelField = ChassisSpeeds.toFieldRelativeSpeeds(targetVelRobot, robotRotation);

            // 2. Predict Linear Velocity (Interpolate in Inches/Second)
            // We are finding the "expected" velocity during the ball's transition
            double predictedVx = robotVelField.vxMetersPerSecond +
                    (targetVelField.vxMetersPerSecond - robotVelField.vxMetersPerSecond) * DRIVE_VEL_PREDICT_ALPHA;

            double predictedVy = robotVelField.vyMetersPerSecond +
                    (targetVelField.vyMetersPerSecond - robotVelField.vyMetersPerSecond) * DRIVE_VEL_PREDICT_ALPHA;

            Vector2d predictedLinearVelIps = new Vector2d(predictedVx, predictedVy);

            // 3. Predict Angular Velocity (Radians/sec)
            double predictedOmegaRadPerSec = robotVelField.omegaRadiansPerSecond +
                    (targetVelField.omegaRadiansPerSecond - robotVelField.omegaRadiansPerSecond) * DRIVE_VEL_PREDICT_ALPHA;

            // 4. Solve
            shotSolution = MathFunctions.VirtualGoalSolver.solve(
                    robotPose,
                    predictedLinearVelIps,
                    predictedOmegaRadPerSec,
                    turret.adjustedGoalPose(),
                    launcher.getTimeOfFlightLUT()
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
}