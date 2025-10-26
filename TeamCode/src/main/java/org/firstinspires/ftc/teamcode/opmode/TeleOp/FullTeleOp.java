package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.SlewRateLimiter;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commandbase.commands.ClearLaunch;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.commandbase.commands.StationaryAimbotFullLaunch;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp(name = "FullTeleOp")
public class FullTeleOp extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;

    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();

    @Override
    public void initialize() {
        // Must have for all opModes
        Constants.OP_MODE_TYPE = OpModeType.TELEOP;

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
        operator = new GamepadEx(gamepad2);

        // Driver controls
        // Reset heading
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> robot.drive.setPose(new Pose2d(robot.drive.getPose().getTranslation(), new Rotation2d())))
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> robot.launcher.setFlywheel(0, false))
        );

        driver.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.launcher.setRamp(false)),
                        new InstantCommand(() -> robot.intake.setPivot(Intake.PivotState.INTAKE)),
                        new InstantCommand(() -> robot.intake.toggleIntake())
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new SetIntake(Intake.MotorState.STOP, Intake.PivotState.HOLD)
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whileActiveContinuous(
                new InstantCommand(() -> robot.intake.setIntake(Intake.MotorState.REVERSE))
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenReleased(
                new InstantCommand(() -> robot.intake.setIntake(Intake.MotorState.FORWARD))
        );

        driver.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.launcher.setRamp(true)).andThen(new WaitCommand(200)),
                        new ClearLaunch()
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new StationaryAimbotFullLaunch()
        );

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> robot.launcher.setFlywheel(LAUNCHER_CLOSE_VELOCITY, false)).alongWith(
                        new InstantCommand(() -> robot.launcher.setHood(MIN_HOOD_ANGLE))
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> robot.launcher.setFlywheel(LAUNCHER_FAR_VELOCITY, false)).alongWith(
                        new InstantCommand(() -> robot.launcher.setHood(MAX_HOOD_ANGLE))
                )
        );
        
        // TODO: Remove after tuning Limelight PID
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenReleased(
                new InstantCommand(() -> robot.turret.setTurret(Turret.TurretState.LIMELIGHT_CONTROL, 0))
        );

    }

    @Override
    public void initialize_loop() {
//        robot.launcher.setMotifState(); // TODO: Make limelight update not every loop

        telemetryData.addData("Launcher Motif State", Turret.motifState);
        telemetryData.update();
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
                double minSpeed = 0.3; // As a fraction of the max speed of the robot
                double speedMultiplier = minSpeed + (1 - minSpeed) * driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

                Rotation2d robotAngle = robot.drive.getPose().getRotation();
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
                        robot.drive.follower.setTarget(new Pose2d(0, 0, robotAngle));
                        headingCorrection = 0;
                    }
                }

                robot.drive.swerve.updateWithTargetVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                driver.getLeftY() * Constants.MAX_VELOCITY * speedMultiplier,
                                -driver.getLeftX() * Constants.MAX_VELOCITY * speedMultiplier,
                                robot.drive.headingLock ? headingCorrection : -driver.getRightX() * Constants.MAX_ANGULAR_VELOCITY * speedMultiplier,
                                robotAngle
                        )
                );
            }
        }
        robot.profiler.end("Swerve Drive");

        robot.profiler.start("High TelemetryData");
        telemetryData.addData("Loop Time", timer.milliseconds());
        timer.reset();

        telemetryData.addData("Heading", robot.drive.getPose().getHeading());
        telemetryData.addData("Robot Pose", robot.drive.getPose());
        telemetryData.addData("Robot Target", robot.drive.follower.getTarget());
        telemetryData.addData("atTarget", robot.drive.follower.atTarget());

        telemetryData.addData("Turret Target", robot.turret.getTarget());
        telemetryData.addData("Turret readyToLaunch", robot.turret.readyToLaunch());
        telemetryData.addData("ATagVisible", robot.turret.ATagVisible);
        telemetryData.addData("Turret State", Turret.turretState);
        telemetryData.addData("Turret Position", robot.turret.getPosition());

        telemetryData.addData("Flywheel Target", robot.launcher.getFlywheelTarget());
        telemetryData.addData("Flywheel Velocity", robot.launchEncoder.getCorrectedVelocity());

        robot.profiler.end("High TelemetryData");
        robot.profiler.start("Low TelemetryData");

        telemetryData.addData("Target Chassis Velocity", robot.drive.swerve.getTargetVelocity());
        telemetryData.addData("FR Module", robot.drive.swerve.getModules()[0].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[0].getPowerTelemetry());
        telemetryData.addData("FL Module", robot.drive.swerve.getModules()[1].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[1].getPowerTelemetry());
        telemetryData.addData("BL Module", robot.drive.swerve.getModules()[2].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[2].getPowerTelemetry());
        telemetryData.addData("BR Module", robot.drive.swerve.getModules()[3].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[3].getPowerTelemetry());

        telemetryData.addData("Sigma", "Polar");
        robot.profiler.end("Low TelemetryData");

        robot.profiler.start("Run + Update");
        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        super.run();
        telemetryData.update();
        robot.controlHub.clearBulkCache();
        robot.expansionHub.clearBulkCache();
        robot.profiler.end("Run + Update");
        robot.profiler.end("Full Loop");
    }

    @Override
    public void end() {
        Constants.END_POSE = robot.drive.getPose();
        robot.exportProfiler(robot.file);
        telemetryData.update();
    }
}