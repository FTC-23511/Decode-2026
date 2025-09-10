package org.firstinspires.ftc.teamcode.globals;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Launcher;


public class Robot extends com.seattlesolvers.solverslib.command.Robot {
    private static final Robot instance = new Robot();
    public static Robot getInstance() {
        return instance;
    }

    public MotorEx FRmotor;
    public MotorEx FLmotor;
    public MotorEx BLmotor;
    public MotorEx BRmotor;

    public MotorEx intakeMotor;
    private MotorEx launchMotorTop;
    private MotorEx launchMotorBottom;

    public MotorGroup launchMotors;

    public CRServoEx FRswervo;
    public CRServoEx FLswervo;
    public CRServoEx BLswervo;
    public CRServoEx BRswervo;

    public ServoEx leftIntakePivot;
    public ServoEx rightIntakePivot;
    public ServoEx depositPivot;

    public RevColorSensorV3 colorSensor;
//    public Limelight3A limelight;

    public GoBildaPinpointDriver pinpoint;
//    public IMU imu;

    public Drive drive;
    public Intake intake;
    public Launcher launcher;

    public void init(HardwareMap hwMap) {
        // Hardware
        FRmotor = new MotorEx(hwMap, "FR").setCachingTolerance(0.01);
        FLmotor = new MotorEx(hwMap, "FL").setCachingTolerance(0.01);;
        BLmotor = new MotorEx(hwMap, "BL").setCachingTolerance(0.01);;
        BRmotor = new MotorEx(hwMap, "BR").setCachingTolerance(0.01);;

        intakeMotor = new MotorEx(hwMap, "intakeMotor").setCachingTolerance(0.01);;
        launchMotorTop = new MotorEx(hwMap, "launchMotorTop").setCachingTolerance(0.01);;
        launchMotorBottom = new MotorEx(hwMap, "launchMotorBottom").setCachingTolerance(0.01);;

        launchMotorBottom.setInverted(true);

        launchMotors = new MotorGroup(launchMotorTop, launchMotorBottom);

        FRswervo = new CRServoEx(hwMap, "FR", new AbsoluteAnalogEncoder(hwMap, "FR").zero(FR_ENCODER_OFFSET), CRServoEx.RunMode.RawPower).setCachingTolerance(0.01);;
        FLswervo = new CRServoEx(hwMap, "FL", new AbsoluteAnalogEncoder(hwMap, "FL").zero(FL_ENCODER_OFFSET), CRServoEx.RunMode.RawPower).setCachingTolerance(0.01);;
        BLswervo = new CRServoEx(hwMap, "BL", new AbsoluteAnalogEncoder(hwMap, "BL").zero(BL_ENCODER_OFFSET), CRServoEx.RunMode.RawPower).setCachingTolerance(0.01);;
        BRswervo = new CRServoEx(hwMap, "BR", new AbsoluteAnalogEncoder(hwMap, "BR").zero(BR_ENCODER_OFFSET), CRServoEx.RunMode.RawPower).setCachingTolerance(0.01);;

        leftIntakePivot = new ServoEx(hwMap, "leftIntakePivot").setCachingTolerance(0.01);;
        rightIntakePivot = new ServoEx(hwMap, "rightIntakePivot").setCachingTolerance(0.01);;
        depositPivot = new ServoEx(hwMap, "depositPivot").setCachingTolerance(0.01);;

        rightIntakePivot.setInverted(true);

        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-75.96, 152.62, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(Pose2d.convertToPose2D(END_POSE, DistanceUnit.INCH, AngleUnit.RADIANS));

        colorSensor = (RevColorSensorV3) hwMap.colorSensor.get("colorSensor");
        colorSensor.enableLed(true);

//        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Subsystems
        drive = new Drive();
        intake = new Intake();
        launcher = new Launcher();

        // Robot/CommandScheduler configurations
        setBulkReading(hwMap, LynxModule.BulkCachingMode.MANUAL);
        register(drive, intake, launcher);
    }

    public void initHasMovement() {

    }

    // Not needed now that we have pinpoint
//    public void initializeImu(HardwareMap hardwareMap) {
//        // IMU orientation
//        imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP));
//        imu.initialize(parameters);
//    }
}