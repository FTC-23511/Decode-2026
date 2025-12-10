package org.firstinspires.ftc.teamcode.globals;

import com.outoftheboxrobotics.photoncore.PhotonCore;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.SensorDigitalDevice;
import com.seattlesolvers.solverslib.hardware.motors.CRServoGroup;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.vision.Camera;

import java.io.File;
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
    public File file;

//    public LynxModule controlHub;
//    public LynxModule expansionHub;
//    public LynxModule servoHub;
//    public ArrayList<LynxModule> hubs;
    public VoltageSensor voltageSensor;
    private double cachedVoltage;
    private ElapsedTime voltageTimer;

    public boolean readyToLaunch = false;

    public MotorEx FRmotor;
    public MotorEx FLmotor;
    public MotorEx BLmotor;
    public MotorEx BRmotor;

    public MotorGroup intakeMotors;

    public MotorGroup launchMotors;
    public Motor.Encoder launchEncoder;

    public CRServoEx FRswervo;
    public CRServoEx FLswervo;
    public CRServoEx BLswervo;
    public CRServoEx BRswervo;

    public CRServoGroup turretServos;
    public AbsoluteAnalogEncoder turretEncoder;

    public ServoEx intakePivotServo;
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

    public SensorDigitalDevice frontDistanceSensor;
    public SensorDigitalDevice backDistanceSensor;

    public void init(HardwareMap hwMap) {
        File logsFolder = new File(AppUtil.FIRST_FOLDER, "logs");
        if (!logsFolder.exists()) logsFolder.mkdirs();

        long timestamp = System.currentTimeMillis();
        file = new File(logsFolder, "profiler-" + timestamp + ".csv");

        profiler = Profiler.builder()
                .factory(new BasicProfilerEntryFactory())
                .exporter(new CSVProfilerExporter(file))
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

        intakeMotors = new MotorGroup(
                new MotorEx(hwMap, "backIntakeMotor")
                        .setCachingTolerance(0.01)
                        .setCurrentAlert(INTAKE_CURRENT_THRESHOLD, CurrentUnit.MILLIAMPS)
                        .setRunMode(Motor.RunMode.RawPower)
                        .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE),
                new MotorEx(hwMap, "frontIntakeMotor")
                        .setCachingTolerance(0.01)
                        .setCurrentAlert(INTAKE_CURRENT_THRESHOLD, CurrentUnit.MILLIAMPS)
                        .setRunMode(Motor.RunMode.RawPower)
                        .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        );

        launchMotors = new MotorGroup(
                new MotorEx(hwMap, "leftLaunchMotor")
                        .setCachingTolerance(0.01),
                new MotorEx(hwMap, "rightLaunchMotor")
                        .setCachingTolerance(0.01)
                        .setInverted(true)
        );

        launchMotors.setRunMode(Motor.RunMode.RawPower);
        launchMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        launchEncoder = new Motor(hwMap, "leftLaunchMotor").encoder;
        launchEncoder.setDirection(Motor.Direction.REVERSE);

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

        turretServos = new CRServoGroup(
                new CRServoEx(hwMap, "leftTurretServo")
                        .setCachingTolerance(0.01)
                        .setRunMode(CRServoEx.RunMode.RawPower),
                new CRServoEx(hwMap, "rightTurretServo")
                        .setCachingTolerance(0.01)
                        .setRunMode(CRServoEx.RunMode.RawPower)
        ).setInverted(true);

        turretEncoder = new AbsoluteAnalogEncoder(hwMap, "turretEncoder")
                .zero(TURRET_ENCODER_OFFSET)
                .setReversed(true);

        intakePivotServo = new ServoEx(hwMap, "intakePivotServo")
                .setInverted(true)
                .setCachingTolerance(0.01);
        hoodServo = new ServoEx(hwMap, "hoodServo").setCachingTolerance(0.001)
                .setInverted(true);
        rampServo = new ServoEx(hwMap, "rampServo").setCachingTolerance(0.001)
                .setInverted(false);;

        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-76.32, 152.62, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(Pose2d.convertToPose2D(END_POSE, DistanceUnit.INCH, AngleUnit.RADIANS));
        pinpoint.setBulkReadScope(GoBildaPinpointDriver.Register.X_POSITION, GoBildaPinpointDriver.Register.Y_POSITION, GoBildaPinpointDriver.Register.H_ORIENTATION);

        frontDistanceSensor = new SensorDigitalDevice(hwMap, "frontDistanceSensor", FRONT_DISTANCE_THRESHOLD);
        backDistanceSensor = new SensorDigitalDevice(hwMap, "backDistanceSensor", BACK_DISTANCE_THRESHOLD);

        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(250);
        limelight.pipelineSwitch(4);
        limelight.start();

        // Subsystems
        drive = new Drive();
        intake = new Intake();
        launcher = new Launcher();
        turret = new Turret();

        // vision
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

        register(drive, intake, launcher, turret);

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
        return 12;
        // this is chopped for loop times
        /*
        if (voltageTimer == null) {
            cachedVoltage = voltageSensor.getVoltage();
        } else if (voltageTimer.milliseconds() > (1.0 / VOLTAGE_SENSOR_POLLING_RATE) * 1000) {
            cachedVoltage = voltageSensor.getVoltage();
        }
        if (((Double) cachedVoltage).isNaN() || cachedVoltage == 0) {
            cachedVoltage = 12;
        }
        return cachedVoltage;
         */
    }

    public void exportProfiler(File file) {
        RobotLog.i("Starting async profiler export to: " + file.getAbsolutePath());

        Thread exportThread = new Thread(() -> {
            try {
                profiler.export();
                profiler.shutdown();
            } catch (Exception e) {
                Log.e("An error occurred", e.toString());
                Log.e(e.toString(), Arrays.toString(e.getStackTrace()));
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

    public void updateLoop(TelemetryData telemetryData) {
        CommandScheduler.getInstance().run();
        telemetryData.update();
        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();
    }
}