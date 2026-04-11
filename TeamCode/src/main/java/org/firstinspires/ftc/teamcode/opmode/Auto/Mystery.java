package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.swerve.coaxial.CoaxialSwerveModule;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.TelemetryEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commandbase.commands.ClearLaunch;
import org.firstinspires.ftc.teamcode.commandbase.commands.DriveTo;
import org.firstinspires.ftc.teamcode.commandbase.commands.PrepDriveTo;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.ArrayList;

//@Config
@Deprecated
@Autonomous(name = "Mystery (far 9 Ball)", preselectTeleOp = "AAAFullTeleOp", group = "Auto")
public class Mystery extends CommandOpMode {
    public ElapsedTime timer;

    TelemetryEx telemetryEx = new TelemetryEx(
            new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())
    );

    private final Robot robot = Robot.getInstance();

    public ArrayList<Pose2d> pathPoses;
    public void generatePath() {
        pathPoses = new ArrayList<>();

        pathPoses.add(new Pose2d(-14.792937399678976, -65.0658105939005, Math.toRadians(90))); // Starting Pose
        pathPoses.add(new Pose2d(-12.413793103448281, -53.71786833855799, Math.toRadians(0))); // Line 1
        pathPoses.add(new Pose2d(-23.02194357366772, -36.33855799373042, Math.toRadians(0))); // Line 2
        pathPoses.add(new Pose2d(-62.52037617554859, -36.11285266457679, Math.toRadians(0))); // Line 3
        pathPoses.add(new Pose2d(-13.090909090909083, -53.49216300940439, Math.toRadians(0))); // Line 4
        pathPoses.add(new Pose2d(-60.327447833065804, -53.97110754414126, Math.toRadians(15))); // Line 5
        pathPoses.add(new Pose2d(-62.068965517241374, -62.746081504702204, Math.toRadians(15))); // Line 6
        pathPoses.add(new Pose2d(-12.865203761755485, -53.71786833855799, Math.toRadians(0))); // Line 7
        pathPoses.add(new Pose2d(-20.08777429467085, -33.40438871473354, Math.toRadians(0))); // Line 8

        if (ALLIANCE_COLOR.equals(AllianceColor.RED)) {
            for (Pose2d pose : pathPoses) {
                pose.mirror();
            }
        }
    }

    @Override
    public void initialize() {
        generatePath();
        timer = new ElapsedTime();

        // Must have for all opModes
        OP_MODE_TYPE = OpModeType.AUTO;
        TESTING_OP_MODE = false;

        // Resets the command scheduler
        super.reset();

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);

        robot.launcher.setHood(MIN_HOOD_SERVO_POS);
        robot.launcher.setRamp(true);
        robot.turret.setTurret(Turret.TurretState.GOAL_LOCK_CONTROL, 0);

        // Schedule the full auto
        // TODO: FIGURE OUT WHY WE NEED A BURNER INSTANT COMMAND
        schedule(
                new SequentialCommandGroup(
                        // init
                        new InstantCommand(),
                        new InstantCommand(() -> robot.turret.setTurret(Turret.TurretState.OFF, 0)),
                        new InstantCommand(() -> robot.drive.setPose(pathPoses.get(0))),

                        // preload
                        pathShoot(1, 3000),

                        // spike 1
                        new DriveTo(pathPoses.get(2)).withTimeout(670),
                        pathIntake(3, 1867, 0.5),
                        pathShoot(4, 2267),

                        // spike 2
                        pathIntake(5, 2267, 0.5),
                        pathShoot(7, 3000),
                        new ClearLaunch(true),

                        new DriveTo(pathPoses.get(8)) // park
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
        robot.updateLoop(telemetryEx);

        // Update any constants that are being updated by FTCDash - used for tuning
        for (CoaxialSwerveModule module : robot.drive.swerve.getModules()) {
            module.setSwervoPIDF(SWERVO_PIDF_COEFFICIENTS);
        }
        ((PIDFController) robot.drive.follower.translationalController).setCoefficients(XY_COEFFICIENTS);
        ((PIDFController) robot.drive.follower.headingController).setCoefficients(HEADING_COEFFICIENTS);

        telemetryEx.addData("Loop Time", timer.milliseconds());
        timer.reset();

        if (PROBLEMATIC_TELEMETRY) {
            robot.profiler.start("High TelemetryData");

            telemetryEx.addData("Heading", robot.drive.getPose().getHeading());
            telemetryEx.addData("Robot Pose", robot.drive.getPose());
            telemetryEx.addData("Turret Position", robot.turret.getPosition());
            telemetryEx.addData("Flywheel Velocity", robot.launchEncoder.getCorrectedVelocity());
            telemetryEx.addData("Intake overCurrent", ((MotorEx) robot.intakeMotors.getMotor()).isOverCurrent());
            telemetryEx.addData("FR Module", robot.drive.swerve.getModules()[0].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[0].getPowerTelemetry());
            telemetryEx.addData("FL Module", robot.drive.swerve.getModules()[1].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[1].getPowerTelemetry());
            telemetryEx.addData("BL Module", robot.drive.swerve.getModules()[2].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[2].getPowerTelemetry());
            telemetryEx.addData("BR Module", robot.drive.swerve.getModules()[3].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[3].getPowerTelemetry());

            robot.profiler.end("High TelemetryData");
        }

        robot.profiler.start("Low TelemetryData");
        telemetryEx.addData("Robot Target", robot.drive.follower.getTarget());
        telemetryEx.addData("atTarget", robot.drive.follower.atTarget());
        telemetryEx.addData("X Error", robot.drive.follower.getError().getTranslation().getX());
        telemetryEx.addData("Y Error", robot.drive.follower.getError().getTranslation().getY());
        telemetryEx.addData("Heading Error", robot.drive.follower.getError().getRotation().getAngle(AngleUnit.RADIANS));

        telemetryEx.addData("Turret State", Turret.turretState);
        telemetryEx.addData("Turret Target", robot.turret.getTarget());
        telemetryEx.addData("Turret readyToLaunch", robot.turret.readyToLaunch());

        telemetryEx.addData("Flywheel Active Control", robot.launcher.getActiveControl());
        telemetryEx.addData("Flywheel Target Ball Velocity", robot.launcher.getTargetFlywheelVelocity());
        telemetryEx.addData("Flywheel Target", robot.launcher.getFlywheelTarget());

        telemetryEx.addData("Intake Motor State", Intake.motorState);
        telemetryEx.addData("Intake Jammed", robot.intake.intakeJammed);

        telemetryEx.addData("Target Chassis Velocity", robot.drive.swerve.getTargetVelocity());

        telemetryEx.addData("Sigma", "Polar");

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        telemetryEx.update();
    }

    @Override
    public void end() {
        END_POSE = robot.drive.getPose();
    }

    public SequentialCommandGroup pathShoot(int pathStartingIndex, long timeout) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SetIntake(Intake.MotorState.FORWARD).beforeStarting(new WaitCommand(410)),
                        new DriveTo(pathPoses.get(pathStartingIndex)).withTimeout(timeout),
                        new InstantCommand(() -> robot.launcher.setFlywheel(LAUNCHER_FAR_VELOCITY, true))
                ),
//                new InstantCommand(() -> robot.turret.setTurret(ANGLE_CONTROL, 0.8 * ALLIANCE_COLOR.getMultiplier())),
                new WaitUntilCommand(() -> robot.turret.readyToLaunch()).withTimeout(500),
                new InstantCommand(() -> robot.readyToLaunch = true),
                new ClearLaunch(true).alongWith(
                        new PrepDriveTo(pathPoses.get(pathStartingIndex + 1))
                )
        );
    }

    public SequentialCommandGroup pathIntake(int pathStartingIndex, long timeout) {
        return pathIntake(pathStartingIndex, timeout, 1.0);
    }

    public SequentialCommandGroup pathIntake(int pathStartingIndex, long timeout, double maxPower) {
        return new SequentialCommandGroup(
                new DriveTo(pathPoses.get(pathStartingIndex)).withTimeout(timeout),
                new SetIntake(Intake.MotorState.FORWARD),

                new DriveTo(pathPoses.get(pathStartingIndex+1), maxPower).withTimeout(2467)
        );
    }
}