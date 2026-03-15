package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.SlewRateLimiter;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commandbase.commands.ClearLaunch;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Disabled
@TeleOp(name = "AAAEventTeleOp")
public class EventTeleop extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;

    private final Robot robot = Robot.getInstance();

    public static double MAX_OUTPUT = 0.5;

    @Override
    public void initialize() {
        // Must have for all opModes
        OP_MODE_TYPE = OpModeType.TELEOP;
        TESTING_OP_MODE = false;

        // Resets the command scheduler
        super.reset();

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);

        driver = new GamepadEx(gamepad1).setJoystickSlewRateLimiters(
                new SlewRateLimiter(STRAFING_SLEW_RATE_LIMIT),
                new SlewRateLimiter(STRAFING_SLEW_RATE_LIMIT),
                new SlewRateLimiter(TURNING_SLEW_RATE_LIMIT),
                null
        );

        // Driver controls
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> robot.drive.setPose(new Pose2d(-24, 24, Math.PI)))
        );

        driver.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.launcher.setRamp(false)),
                        new InstantCommand(() -> robot.intake.setIntake(Intake.MotorState.FORWARD))
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.CIRCLE).whenReleased(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.launcher.setRamp(false)),
                        new InstantCommand(() -> robot.intake.setIntake(Intake.MotorState.STOP))
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.readyToLaunch = true),
                        new InstantCommand(() -> robot.launcher.setActiveControl(true)),
                        new InstantCommand(() -> robot.launcher.setRamp(true)),
                        new ClearLaunch(false)
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> robot.launcher.setFlywheel(LAUNCHER_CLOSE_VELOCITY, false)).alongWith(
                        new InstantCommand(() -> robot.launcher.setHood(MIN_HOOD_ANGLE))
                )
        );
    }

    @Override
    public void initialize_loop() {
        robot.drive.setPose(END_POSE);
    }

    @Override
    public void run() {
        robot.profiler.start("Full Loop");
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            robot.initHasMovement();
            timer = new ElapsedTime();
        }

        robot.profiler.start("Swerve Drive");
        if (CommandScheduler.getInstance().isAvailable(robot.drive)) {
            // Drive the robot
            if (driver.isDown(GamepadKeys.Button.START)) {
                robot.drive.swerve.updateWithXLock();
            } else {
                robot.drive.swerve.setMaxSpeed(MAX_OUTPUT);
                // As a fraction of the max speed of the robot
                double speedMultiplier = 0.35;

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
                                new Rotation2d(robotAngle.getAngle(AngleUnit.RADIANS) + (ALLIANCE_COLOR.equals(AllianceColor.BLUE) ? Math.PI : 0))
                        )
                );
            }
        }


        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        robot.updateLoop(null);
    }

    @Override
    public void end() {
        Constants.END_POSE = robot.drive.getPose();
    }
}