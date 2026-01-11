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

@TeleOp(name = "AAAFullTeleOp")
public class FullTeleOp extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    public GamepadEx driver, operator;
    public ElapsedTime timer;
    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    @Override
    public void initialize() {
        Constants.OP_MODE_TYPE = OpModeType.TELEOP;
        super.reset();
        robot.init(hardwareMap);

        driver = new GamepadEx(gamepad1).setJoystickSlewRateLimiters(
                new SlewRateLimiter(STRAFING_SLEW_RATE_LIMIT),
                new SlewRateLimiter(STRAFING_SLEW_RATE_LIMIT),
                new SlewRateLimiter(TURNING_SLEW_RATE_LIMIT),
                null
        );
        operator = new GamepadEx(gamepad2);

        // --- SHOOTING SEQUENCES ---

        // SQUARE: Medium Shot (1200 ticks, 28 deg)
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

// CIRCLE: Wait for flywheel to reach reverse speed, then intake
        driver.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.launcher.setFlywheel(-500, true)),
                        new WaitUntilCommand(() -> robot.launcher.flywheelReady()),
                        new InstantCommand(() -> robot.intake.setIntake(Intake.MotorState.FORWARD))
                )
        );




        // TRIANGLE: High Shot (1700 ticks, 10 deg)
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

        // --- UTILITY CONTROLS ---

        // LEFT STICK: Reset Heading
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

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(new UninterruptibleCommand(new CancelCommand()));
    }

    @Override
    public void run() {
        if (timer == null) {
            robot.initHasMovement();
            timer = new ElapsedTime();
        }

        handleDrive();

        // --- TELEMETRY ---
        telemetryData.addData("Loop Time", timer.milliseconds());
        telemetryData.addData("Flywheel Target", robot.launcher.getTargetVelocity()); // Ensure this method exists in Launcher
        telemetryData.addData("Flywheel Actual", robot.launchEncoder.getCorrectedVelocity());
        telemetryData.addData("Flywheel Ready", robot.launcher.flywheelReady());
        // Added Launcher Power check
        telemetryData.addData("Launcher Power", robot.launcher.getPower());

        timer.reset();
        robot.updateLoop(telemetryData);
    }

    private void handleDrive() {
        if (!CommandScheduler.getInstance().isAvailable(robot.drive)) return;

        double speedMultiplier = 0.5 + (0.5 * driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        Pose2d robotPose = robot.drive.getPose();
        double headingCorrection = 0;

        if (Math.abs(driver.getRightX()) < JOYSTICK_DEAD_ZONE && !robot.drive.headingLock) {
            robot.drive.headingLock = true;
            robot.drive.follower.setTarget(new Pose2d(0, 0, robotPose.getRotation()));
        } else if (Math.abs(driver.getRightX()) > JOYSTICK_DEAD_ZONE) {
            robot.drive.headingLock = false;
        }

        if (robot.drive.headingLock) {
            headingCorrection = robot.drive.follower.calculate(new Pose2d(0, 0, robotPose.getRotation())).omegaRadiansPerSecond;
        }

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
