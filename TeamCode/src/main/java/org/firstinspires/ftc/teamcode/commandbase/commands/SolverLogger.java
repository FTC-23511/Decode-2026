package org.firstinspires.ftc.teamcode.commandbase.commands;

import android.os.Environment;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Robot;
import org.firstinspires.ftc.teamcode.opmode.TeleOp.FullTeleOpLogging;

import java.io.IOException;
import java.util.logging.FileHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;


public class SolverLogger {

    String logFileName;
    private FileHandler logFileHandler;

    public ElapsedTime timer;

    private Logger logger;

    public GamepadEx driver;


    private final Robot robot = Robot.getInstance();


    public SolverLogger(String fileName)  {
        logFileName = fileName;
        timer = new ElapsedTime();

    }

    public boolean init() {
        if (logger != null) {
            return true;
        }

        try {
            // Create a FileHandler to write logs to a file named "my_application.log"
            // The 'true' argument means to append to the file if it exists,
            // otherwise, a new file will be created.
            String fullLogPath = Environment.getExternalStorageDirectory().getPath() + "/FIRST/" + logFileName;
            logFileHandler = new FileHandler(fullLogPath, false);

            //Simple Formatter
            SimpleFormatter formatter = new SimpleFormatter();
            logFileHandler.setFormatter(formatter);

            logger = Logger.getLogger(FullTeleOpLogging.class.getName());
            logger.addHandler(logFileHandler);
            logger.setLevel(Level.INFO);

            logger = Logger.getLogger(FullTeleOpLogging.class.getName());
            // Add the FileHandler to the logger.
            logger.addHandler(logFileHandler);

            logger.setLevel(Level.INFO);

        } catch (SecurityException | IOException e) {
            e.printStackTrace();
            return false;
        }

        return true;
    }

    public void deinit() {
        logger.info("Deinitialize the logger");
        logger = null;
        logFileHandler.close();
        logFileHandler = null;
    }

    public void log() {
        String str = "";
//        double botHeading = robot.getYawDegrees();


        // log format
        // [Name:Type:Value];
        // E.g.
        // Timestamp:int64:value;Pose:Tuple:{a,b,c};
        str += "Loop Time:Double:" + timer.milliseconds();

        str += ";Heading:Double:" + robot.drive.getPose().getHeading();
        str += ";Robot Pose:Pose2D;" + robot.drive.getPose();
        str += ";Turret Position:Double;" + robot.turret.getPosition();
        str += ";Flywheel Velocity:Double:" + robot.launchEncoder.getCorrectedVelocity();
        str += ";Intake overCurrent:Boolean:" + ((MotorEx) robot.intakeMotors.getMotor()).isOverCurrent();
        str += ";FR Module Target Velocity0:Vector2D:" + robot.drive.swerve.getModules()[0].getTargetVelocity();
        str += ";FR Power Telemetry0:Power Telemetry:" + robot.drive.swerve.getModules()[0].getPowerTelemetry();
        str += ";FL Module Target Velocity1:Vector2D:" +  robot.drive.swerve.getModules()[1].getTargetVelocity();
        str += ";FL Power Telemetry1:Power Telemetry:" + robot.drive.swerve.getModules()[1].getPowerTelemetry();
        str += ";BL Module Target Velocity2:Vector2D:" + robot.drive.swerve.getModules()[2].getTargetVelocity();
        str += ";BL Power Telemetry2:Power Telemetry:" + robot.drive.swerve.getModules()[2].getPowerTelemetry();
        str += ";BR Module Target Velocity3:Vector2D:" + robot.drive.swerve.getModules()[3].getTargetVelocity();
        str += ";BR Power Telemetry3:Power Telemetry:" + robot.drive.swerve.getModules()[3].getPowerTelemetry();

        str += ";Robot Target:Pose2D:" + robot.drive.follower.getTarget();
        str += ";atTarget:Boolean:" + robot.drive.follower.atTarget();

        str += ";Turret State:String:" + Turret.turretState;
        str += ";Turret Target:Double:" + robot.turret.getTarget();
        str += ";Turret readyToLaunch:Boolean:" + robot.turret.readyToLaunch();
        str += ";turretPose:Pose2D:" + robot.turret.getTurretPose();
        str += ";Wall Angle:Double:" + robot.camera.getMedianWallAngle();

        str += ";Flywheel Active Control:Boolean:" + robot.launcher.getActiveControl();
        str += ";Flywheel Target Ball Velocity:Double:" + robot.launcher.getTargetFlywheelVelocity();
        str += ";Flywheel Target:Double:" + robot.launcher.getFlywheelTarget();

        str += ";Intake Motor State:String:" + Intake.motorState;
        str += ";Intake Jammed:Boolean:" + robot.intake.intakeJammed;

        str += ";Target Chassis Velocity:ChassisSpeeds:" + robot.drive.swerve.getTargetVelocity().toString();


        logger.info(str);
    }


}
