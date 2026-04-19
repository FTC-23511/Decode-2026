package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.globals.Constants.ALLIANCE_COLOR;
import static org.firstinspires.ftc.teamcode.globals.Constants.JOYSTICK_DEAD_ZONE;
import static org.firstinspires.ftc.teamcode.globals.Constants.MAX_TELEOP_HEADING_CORRECTION_VEL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.drivebase.swerve.coaxial.CoaxialSwerveModule;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.util.TelemetryEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp(name = "SwerveTeleOp", group = "TeleOp")
public class SwerveTeleOp extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;

    TelemetryEx telemetryEx = new TelemetryEx(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    public static double MIN_OUTPUT = 0.3; // As a fraction of the max speed of the robot
    public static double MAX_OUTPUT = 1; // As a fraction of the max speed of the robot

    private final Robot robot = Robot.getInstance();

    @Override
    public void initialize() {
        // Must have for all opModes
        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;
        Constants.TESTING_OP_MODE = true;

        // Resets the command scheduler
        super.reset();

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        // Driver controls
        // Reset heading
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> robot.drive.setPose(new Pose2d()))
        );
    }

    @Override
    public void run() {
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            robot.initHasMovement();
            timer = new ElapsedTime();
        }

        // Update any constants that are being updated by FTCDash
        for (CoaxialSwerveModule module : robot.drive.swerve.getModules()) {
            module.setSwervoPIDF(Constants.SWERVO_PIDF_COEFFICIENTS);
        }

        if (CommandScheduler.getInstance().isAvailable(robot.drive)) {
            // Drive the robot
            if (driver.isDown(GamepadKeys.Button.START)) {
                robot.drive.swerve.updateWithXLock();
            } else {
                robot.drive.swerve.setMaxSpeed(MAX_OUTPUT);
                double speedMultiplier = MIN_OUTPUT + (1 - MIN_OUTPUT) * driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

                Pose2d robotPose = robot.drive.getPose();
                Rotation2d robotAngle = robotPose.getRotation();
                double headingCorrection = 0;

                if (Math.abs(driver.getRightX()) < JOYSTICK_DEAD_ZONE && !robot.drive.headingLock) {
                    robot.drive.headingLock = true;
                    robot.drive.follower.setTarget(new Pose2d(0, 0, robotAngle));
                } else if (Math.abs(driver.getRightX()) > JOYSTICK_DEAD_ZONE) {
                    robot.drive.headingLock = false;
                } else if (robot.drive.headingLock) {
                    headingCorrection = robot.drive.follower.calculate(new Pose2d(0, 0, robotAngle)).omegaRadiansPerSecond;
                    if (robot.drive.follower.atTarget()) {
                        headingCorrection = 0;
                    } else if (Math.abs(headingCorrection) > MAX_TELEOP_HEADING_CORRECTION_VEL) {
                        robot.drive.follower.setTarget(new Pose2d(robotPose.getTranslation(), robotAngle));
                        headingCorrection = 0;
                    }
                }

                robot.drive.swerve.updateWithTargetVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                driver.getLeftY() * Constants.MAX_DRIVE_VELOCITY * speedMultiplier,
                                -driver.getLeftX() * Constants.MAX_DRIVE_VELOCITY * speedMultiplier,
                                robot.drive.headingLock ? headingCorrection : -driver.getRightX() * Constants.MAX_ANGULAR_VELOCITY * speedMultiplier,
                                new Rotation2d(robotAngle.getAngle(AngleUnit.RADIANS) + (ALLIANCE_COLOR.equals(Constants.AllianceColor.BLUE) ? Math.PI : 0))
                        )
                );
            }
        }

        telemetryEx.addData("Loop Time", timer.milliseconds());
        timer.reset();

        telemetryEx.addData("Heading", robot.drive.getPose().getHeading());
        telemetryEx.addData("Robot Pose", robot.drive.getPose());

        telemetryEx.addData("Target Chassis Velocity", robot.drive.swerve.getTargetVelocity());
        telemetryEx.addData("FR Module", robot.drive.swerve.getModules()[0].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[0].getPowerTelemetry());
        telemetryEx.addData("FL Module", robot.drive.swerve.getModules()[1].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[1].getPowerTelemetry());
        telemetryEx.addData("BL Module", robot.drive.swerve.getModules()[2].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[2].getPowerTelemetry());
        telemetryEx.addData("BR Module", robot.drive.swerve.getModules()[3].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[3].getPowerTelemetry());

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        robot.updateLoop(telemetryEx);
    }

    @Override
    public void end() {
        Constants.END_POSE = robot.drive.getPose();
        telemetryEx.update();
        robot.exportProfiler(robot.profilerFile, robot.logCatFile);
    }
}