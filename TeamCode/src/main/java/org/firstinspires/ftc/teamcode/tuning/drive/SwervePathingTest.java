package org.firstinspires.ftc.teamcode.tuning.drive;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.swerve.coaxial.CoaxialSwerveModule;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.TelemetryEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commandbase.commands.DriveTo;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.ArrayList;

@Config
@Autonomous(name = "SwervePathingTest", group = "Drive")
public class SwervePathingTest extends CommandOpMode {
    public ElapsedTime timer;

    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    TelemetryEx telemetryEx = new TelemetryEx(multipleTelemetry);

    private final Robot robot = Robot.getInstance();
    public static boolean FOLLOW_PREPROGRAMMED_PATHS = false;
    public static double xTarget = 0;
    public static double yTarget = 0;
    public static double headingTarget = 0;

    public ArrayList<Pose2d> pathPoses;
    public void generatePath() {
        pathPoses = new ArrayList<Pose2d>();

        pathPoses.add(new Pose2d(0, 0, 0)); // Starting Pose
        pathPoses.add(new Pose2d(24, 24, Math.PI/2)); // Line 1
        pathPoses.add(new Pose2d(0, 0, 0)); // Line 2
    }

    @Override
    public void initialize() {
        generatePath();
        timer = new ElapsedTime();

        // Must have for all opModes
        OP_MODE_TYPE = OpModeType.AUTO;
        TESTING_OP_MODE = true;

        // Resets the command scheduler
        super.reset();

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);

//        robot.drive.follower.setSlewRateLimiters(
//                new SlewRateLimiter(AUTO_STRAFING_SLEW_RATE_LIMIT),
//                new SlewRateLimiter(AUTO_STRAFING_SLEW_RATE_LIMIT),
//                new SlewRateLimiter(AUTO_TURNING_SLEW_RATE_LIMIT)
//        );

        // Schedule the full auto
        robot.drive.setPose(pathPoses.get(0));

        schedule(
                new InstantCommand(),
                new ConditionalCommand(
                        new SequentialCommandGroup(
//                                new DriveTo(pathPoses.get(1)),
//                                new DriveTo(pathPoses.get(2))
                        ),
                        new RunCommand(
                                () -> schedule(new DriveTo(new Pose2d(xTarget, yTarget, new Rotation2d(headingTarget))))
                        ),
                        () -> FOLLOW_PREPROGRAMMED_PATHS
                )
        );
    }

    @Override
    public void initialize_loop() {
        telemetryEx.addData("ALLIANCE_COLOR", ALLIANCE_COLOR);
        telemetryEx.update();
    }

    @Override
    public void run() {
        // DO NOT REMOVE
        robot.updateLoop(null);
        robot.drive.periodic();

        // Update any constants that are being updated by FTCDash - used for tuning
        for (CoaxialSwerveModule module : robot.drive.swerve.getModules()) {
            module.setSwervoPIDF(SWERVO_PIDF_COEFFICIENTS);
        }
        ((PIDFController) robot.drive.follower.translationalController).setCoefficients(XY_COEFFICIENTS);
        ((PIDFController) robot.drive.follower.headingController).setCoefficients(AUTO_HEADING_COEFFICIENTS);

        if (PROBLEMATIC_TELEMETRY) {
            robot.profiler.start("High TelemetryData");

            telemetryEx.addData("Heading", robot.drive.getPose().getHeading());
            telemetryEx.addData("Robot Pose", robot.drive.getPose());
            telemetryEx.addData("Robot Target", robot.drive.follower.getTarget());
            telemetryEx.addData("Target Chassis Velocity", robot.drive.swerve.getTargetVelocity());
            telemetryEx.addData("Intake overCurrent", ((MotorEx) robot.intakeMotor.getMotor()).isOverCurrent());
            telemetryEx.addData("FR Module", robot.drive.swerve.getModules()[0].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[0].getPowerTelemetry());
            telemetryEx.addData("FL Module", robot.drive.swerve.getModules()[1].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[1].getPowerTelemetry());
            telemetryEx.addData("BL Module", robot.drive.swerve.getModules()[2].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[2].getPowerTelemetry());
            telemetryEx.addData("BR Module", robot.drive.swerve.getModules()[3].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[3].getPowerTelemetry());

            telemetryEx.update();

            robot.profiler.end("High TelemetryData");
        }

        robot.profiler.start("Low TelemetryData");

        multipleTelemetry.addData("atTarget", robot.drive.follower.atTarget());
        multipleTelemetry.addData("X Error", robot.drive.follower.getError().getTranslation().getX());
        multipleTelemetry.addData("Y Error", robot.drive.follower.getError().getTranslation().getY());
        multipleTelemetry.addData("Heading Error", robot.drive.follower.getError().getRotation().getAngle(AngleUnit.RADIANS));
        multipleTelemetry.addData("Loop Times", timer.milliseconds());
        multipleTelemetry.update();

        timer.reset();

        robot.profiler.end("Low TelemetryData");
    }

    @Override
    public void end() {
        END_POSE = robot.drive.getPose();
    }

}