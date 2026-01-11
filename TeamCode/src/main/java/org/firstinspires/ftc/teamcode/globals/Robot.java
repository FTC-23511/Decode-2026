package org.firstinspires.ftc.teamcode.globals;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import static org.firstinspires.ftc.teamcode.globals.Constants.*;
import android.util.Log;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
import java.io.File;
import java.util.Arrays;
import dev.nullftc.profiler.Profiler;
import dev.nullftc.profiler.entry.BasicProfilerEntryFactory;
import dev.nullftc.profiler.exporter.CSVProfilerExporter;

/**
 * Robot Hardware Configuration (Singleton)
 * This class centralizes all hardware initializations, PhotonCore optimizations,
 * and global subsystem instances.
 */
public class Robot extends com.seattlesolvers.solverslib.command.Robot {
    // Singleton Pattern: Ensures only one instance of hardware exists at a time
    private static final Robot instance = new Robot();
    public static Robot getInstance() {
        return instance;
    }

    // Profiling and logging
    public Profiler profiler;
    public File file;

    // Power management
    public VoltageSensor voltageSensor;
    private double cachedVoltage;
    private ElapsedTime voltageTimer;

    // --- SWERVE DRIVE MOTORS ---
    public MotorEx FRmotor, FLmotor, BLmotor, BRmotor;

    // --- SUBSYSTEM HARDWARE GROUPS ---
    public MotorGroup intakeMotors;
    public MotorGroup launchMotors;
    public Motor.Encoder launchEncoder;

    // --- SWERVE STEERING SERVOS (CR) ---
    public CRServoEx FRswervo, FLswervo, BLswervo, BRswervo;

    // --- AUXILIARY SERVOS & SENSORS ---
    public ServoEx intakePivotServo;
    public ServoEx hoodServo;
    public ServoEx rampServo;
    public Limelight3A limelight;
    public GoBildaPinpointDriver pinpoint;

    // --- SUBSYSTEM INSTANCES ---
    public Drive drive;
    public Intake intake;
    public Launcher launcher;

    // --- DISTANCE SENSORS ---
    public SensorDigitalDevice frontDistanceSensor, backDistanceSensor;
    public AnalogInput distanceSensor;

    /**
     * Initializes all hardware components.
     * @param hwMap The hardwareMap provided by the OpMode.
     */
    public void init(HardwareMap hwMap) {
        // --- PROFILER SETUP ---
        // Creates a CSV log file in the internal storage for performance analysis
        File logsFolder = new File(AppUtil.FIRST_FOLDER, "logs");
        if (!logsFolder.exists()) logsFolder.mkdirs();
        long timestamp = System.currentTimeMillis();
        file = new File(logsFolder, "profiler-" + timestamp + ".csv");

        profiler = Profiler.builder()
                .factory(new BasicProfilerEntryFactory())
                .exporter(new CSVProfilerExporter(file))
                .debugLog(false)
                .build();

        // --- PHOTONCORE OPTIMIZATIONS ---
        // MANUAL bulk caching reduces I/O latency by reading all sensors once per loop
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8); // Overclock command execution
        PhotonCore.enable();

        // --- DRIVE MOTORS ---
        voltageSensor = hwMap.voltageSensor.iterator().next();
        FRmotor = new MotorEx(hwMap, "FRM").setCachingTolerance(0.01);
        FLmotor = new MotorEx(hwMap, "FLM").setCachingTolerance(0.01);
        BLmotor = new MotorEx(hwMap, "BLM").setCachingTolerance(0.01);
        BRmotor = new MotorEx(hwMap, "BRM").setCachingTolerance(0.01);

        // --- INTAKE MOTORS ---
        intakeMotors = new MotorGroup(
                new MotorEx(hwMap, "leftIntakeMotor")
                        .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
                        .setInverted(true),
                new MotorEx(hwMap, "rightIntakeMotor")
                        .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        );

        // --- LAUNCHER MOTORS ---
        launchMotors = new MotorGroup(
                new MotorEx(hwMap, "leftLaunchMotor").setInverted(true),
                new MotorEx(hwMap, "rightLaunchMotor")
        );
        launchMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT); // Allows free spin-down

        // Separate encoder instance for the flywheel to get high-frequency velocity data
        launchEncoder = new Motor(hwMap, "rightLaunchMotor").encoder;

        // --- SWERVE STEERING (SWERVOS) ---
        // Combines a Continuous Rotation Servo with an Absolute Analog Encoder
        FRswervo = new CRServoEx(hwMap, "FRS", new AbsoluteAnalogEncoder(hwMap, "FRE").setReversed(true).zero(FR_ENCODER_OFFSET), CRServoEx.RunMode.RawPower);
        FLswervo = new CRServoEx(hwMap, "FLS", new AbsoluteAnalogEncoder(hwMap, "FLE").setReversed(true).zero(FL_ENCODER_OFFSET), CRServoEx.RunMode.RawPower);
        BLswervo = new CRServoEx(hwMap, "BLS", new AbsoluteAnalogEncoder(hwMap, "BLE").setReversed(true).zero(BL_ENCODER_OFFSET), CRServoEx.RunMode.RawPower);
        BRswervo = new CRServoEx(hwMap, "BRS", new AbsoluteAnalogEncoder(hwMap, "BRE").setReversed(true).zero(BR_ENCODER_OFFSET), CRServoEx.RunMode.RawPower);

        // Ensure all steering pods are inverted for clockwise/counter-clockwise consistency
        FRswervo.setInverted(true);
        FLswervo.setInverted(true);
        BLswervo.setInverted(true);
        BRswervo.setInverted(true);

        // --- ODOMETRY (PINPOINT) ---
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint")
                .setOffsets(-76.32, 152.62, DistanceUnit.MM)
                .setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                .setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED)
                .resetPosAndIMU();

        // --- SUBSYSTEM REGISTRATION ---
        // Creates instances of the actual logic classes
        drive = new Drive();
        intake = new Intake();
        launcher = new Launcher();

        // Link subsystems to the CommandScheduler
        register(drive, intake, launcher);

        if (OP_MODE_TYPE.equals(OpModeType.AUTO)) {
            initHasMovement();
        }
    }

    /**
     * Resets subsystems that involve physical movement (motors/servos).
     */
    public void initHasMovement() {
        drive.init();
        intake.init();
        launcher.init();
    }

    /**
     * Safely exports the profiler data to a file on a background thread
     * to prevent blocking the main robot loop during a match.
     */
    public void exportProfiler(File file) {
        RobotLog.i("Starting async profiler export to: " + file.getAbsolutePath());
        Thread exportThread = new Thread(() -> {
            try {
                profiler.export();
                profiler.shutdown();
            } catch (Exception e) {
                Log.e("Profiler Error", e.toString());
            }
        });
        exportThread.setDaemon(true);
        exportThread.start();
    }

    /**
     * Main control loop logic.
     * 1. Runs the Command Scheduler
     * 2. Updates Telemetry
     * 3. Clears PhotonCore Bulk Caches to prepare for the next loop's hardware reads
     */
    public void updateLoop(TelemetryData telemetryData) {
        CommandScheduler.getInstance().run();
        telemetryData.update();

        // Crucial: This allows the next loop to fetch fresh data from the Rev Hubs
        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();
    }
}
