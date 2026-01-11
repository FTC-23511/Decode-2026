package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.*;
import com.seattlesolvers.solverslib.gamepad.*;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.util.TelemetryData;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commandbase.commands.CancelCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

/**
 * Main TeleOp OpMode for the 2025-2026 Season.
 * This class uses the SolversLib Command Framework to handle concurrent tasks
 * such as driving, intaking, and automated shooting sequences.
 */
@TeleOp(name = "AAAFullTeleOp")
public class FullTeleOp extends CommandOpMode {
    // Singleton instance of the robot hardware configuration
    private final Robot robot = Robot.getInstance();

    public GamepadEx driver, operator;
    public ElapsedTime timer;

    // Telemetry setup to mirror data to both the Driver Station and FTC Dashboard
    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    @Override
    public void initialize() {
        // Set state to TeleOp to handle sensor polling rates and pose handoffs correctly
        Constants.OP_MODE_TYPE = OpModeType.TELEOP;

        // Reset the command scheduler and initialize hardware
        super.reset();
        robot.init(hardwareMap);

        // Configure driver gamepad with Slew Rate Limiters to prevent jerky movement/tipping
        driver = new GamepadEx(gamepad1).setJoystickSlewRateLimiters(
                new SlewRateLimiter(STRAFING_SLEW_RATE_LIMIT),
                new SlewRateLimiter(STRAFING_SLEW_RATE_LIMIT),
                new SlewRateLimiter(TURNING_SLEW_RATE_LIMIT),
                null
        );
        operator = new GamepadEx(gamepad2);

        // --- SHOOTING SEQUENCES ---

        // SQUARE: Medium Shot Logic
        // 1. Stop intake 2. Set hood to 28 degrees 3. Spin flywheel to 1200 ticks/s
        // 4. Wait for PID to settle 5. Feed the ball into the launcher (Transfer)
        driver.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new SequentialCommandGroup(
                        Intake.ActiveStopIntake(),
                        new com.seattlesolvers.solverslib.command.WaitCommand(100),
                        new InstantCommand(() -> {
                            robot.launcher.setHood(28);
                            robot.launcher.setFlywheel(1200, true);
                        }),
                        new WaitUntilCommand(() -> robot.launcher.flywheelReady()),
                        new InstantCommand(() -> robot.intake.setIntake(Intake.MotorState.TRANSFER))
                )
        );

        // CIRCLE: Reverse Intake Logic
        // Spins the flywheel backwards at 500 ticks/s to clear jams or assist in intaking.
        driver.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.launcher.setFlywheel(-500, true)),
                        new WaitUntilCommand(() -> robot.launcher.flywheelReady()),
                        new InstantCommand(() -> robot.intake.setIntake(Intake.MotorState.FORWARD))
                )
        );

        // TRIANGLE: High Shot Logic
        // Operates similarly to the Square button but with higher velocity (1700) and lower hood angle (10).
        driver.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new SequentialCommandGroup(
                        Intake.ActiveStopIntake(),
                        new com.seattlesolvers.solverslib.command.WaitCommand(100),
                        new InstantCommand(() -> {
                            robot.launcher.setHood(10);
                            robot.launcher.setFlywheel(1700, true);
                        }),
                        new WaitUntilCommand(() -> robot.launcher.flywheelReady()),
                        new InstantCommand(() -> robot.intake.setIntake(Intake.MotorState.TRANSFER))
                )
        );

        // --- SHOOTER MANUAL ADJUSTMENTS (D-PAD) ---
        // Allows the operator to fine-tune shot parameters during the match.

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(robot.launcher::adjustFlywheelSpeedUp, robot.launcher)
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(robot.launcher::adjustFlywheelSpeedDown, robot.launcher)
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(robot.launcher::adjustHoodUp, robot.launcher)
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(robot.launcher::adjustHoodDown, robot.launcher)
        );

        // --- UTILITY CONTROLS ---

        // LEFT STICK: Re-zero the field-centric heading.
        // If Blue alliance, it offsets by 180 degrees (PI) to keep "Forward" away from the driver.
        driver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                new SequentialCommandGroup(
                        new ConditionalCommand(
                                new InstantCommand(() -> robot.drive.setPose(new Pose2d(robot.drive.getPose().getTranslation(), new Rotation2d(Math.PI)))),
                                new InstantCommand(() -> robot.drive.setPose(new Pose2d(robot.drive.getPose().getTranslation(), new Rotation2d()))),
                                () -> ALLIANCE_COLOR.equals(AllianceColor.BLUE)
                        ),
                        new InstantCommand(() -> robot.drive.unsureXY = true)
                )
        );

        // RIGHT STICK: Emergency stop for all active commands.
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(new UninterruptibleCommand(new CancelCommand()));
    }

    @Override
    public void run() {
        // Initialization check to start timers and pinpoint odometry polling
        if (timer == null) {
            robot.initHasMovement();
            timer = new ElapsedTime();
        }

        // Handle the manual drivetrain inputs
        handleDrive();

        // --- TELEMETRY ---
        telemetryData.addData("Loop Time", timer.milliseconds());
        telemetryData.addData("Flywheel Target", robot.launcher.getTargetVelocity());
        telemetryData.addData("Flywheel Actual", robot.launchEncoder.getCorrectedVelocity());
        telemetryData.addData("Flywheel Ready", robot.launcher.flywheelReady());
        telemetryData.addData("Launcher Power", robot.launcher.getPower());

        timer.reset();

        // Executes the scheduled commands and clears bulk sensor caches
        robot.updateLoop(telemetryData);
    }

    /**
     * Translates gamepad stick inputs into field-relative swerve movement.
     * Includes "Heading Lock" logic to keep the robot pointed in a specific direction when not turning.
     */
    private void handleDrive() {
        // If a command (like an Auto-align) is using the drive, don't allow manual input
        if (!CommandScheduler.getInstance().isAvailable(robot.drive)) return;

        // Turbo mode: Holding the Right Trigger increases speed from 50% to 100%
        double speedMultiplier = 0.5 + (0.5 * driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        Pose2d robotPose = robot.drive.getPose();
        double headingCorrection = 0;

        // Heading Lock Logic:
        // If the right stick is centered, the robot uses a PID controller to maintain its current heading.
        if (Math.abs(driver.getRightX()) < JOYSTICK_DEAD_ZONE && !robot.drive.headingLock) {
            robot.drive.headingLock = true;
            robot.drive.follower.setTarget(new Pose2d(0, 0, robotPose.getRotation()));
        } else if (Math.abs(driver.getRightX()) > JOYSTICK_DEAD_ZONE) {
            robot.drive.headingLock = false;
        }

        // Calculate the power needed to stay at the target heading
        if (robot.drive.headingLock) {
            headingCorrection = robot.drive.follower.calculate(new Pose2d(0, 0, robotPose.getRotation())).omegaRadiansPerSecond;
        }

        // Convert field-relative stick inputs into robot-relative velocities for the swerve modules
        robot.drive.swerve.updateWithTargetVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        driver.getLeftY() * MAX_DRIVE_VELOCITY * speedMultiplier,
                        -driver.getLeftX() * MAX_DRIVE_VELOCITY * speedMultiplier,
                        robot.drive.headingLock ? headingCorrection : -driver.getRightX() * MAX_ANGULAR_VELOCITY * speedMultiplier,
                        new Rotation2d(robotPose.getRotation().getAngle(AngleUnit.RADIANS) + (ALLIANCE_COLOR == AllianceColor.BLUE ? Math.PI : 0))
                )
        );
    }
}
