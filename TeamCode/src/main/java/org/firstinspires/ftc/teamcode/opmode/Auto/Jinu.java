package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.ANGLE_CONTROL;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.GOAL_LOCK_CONTROL;
import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.swerve.coaxial.CoaxialSwerveModule;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commandbase.commands.ClearLaunch;
import org.firstinspires.ftc.teamcode.commandbase.commands.DriveTo;
import org.firstinspires.ftc.teamcode.commandbase.commands.PrepDriveTo;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.ArrayList;

@Config
@Autonomous(name = "Jinu (close gate auto)", preselectTeleOp = "AAAFullTeleOp")
public class Jinu extends CommandOpMode {
    public ElapsedTime timer;

    TelemetryData telemetryData = new TelemetryData(
            new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())
    );

    private final Robot robot = Robot.getInstance();

    public ArrayList<Pose2d> pathPoses;
    public void generatePath() {
        pathPoses = new ArrayList<>();

        pathPoses.add(new Pose2d(-48.947813822284914, 57.98589562764457, Math.toRadians(53))); // Starting Pose
        pathPoses.add(new Pose2d(-13.607898448519048, 12.490832157968967, Math.toRadians(20))); // Line 1
        pathPoses.add(new Pose2d(-55.24400564174894, 12.490832157968967, Math.toRadians(0))); // Line 2
        pathPoses.add(new Pose2d(-49.150916784203105, 1.1170662905500706, Math.toRadians(0))); // Line 3
        pathPoses.add(new Pose2d(-58.40288924558587, 1.1170662905500706, Math.toRadians(0))); // Line 4
        pathPoses.add(new Pose2d(-13.607898448519048, 12.490832157968967, Math.toRadians(20))); // Line 5
        pathPoses.add(new Pose2d(-24.16925246826516, -12.490832157968967, Math.toRadians(0))); // Line 6
        pathPoses.add(new Pose2d(-63.10112359550561, -12.597110754414118, Math.toRadians(0))); // Line 7
        pathPoses.add(new Pose2d(-34.1212976022567, -12.490832157968967, Math.toRadians(0))); // Line 8
        pathPoses.add(new Pose2d(-13.607898448519048, 12.490832157968967, Math.toRadians(20))); // Line 9
        pathPoses.add(new Pose2d(-24.73194221508828, -36.173354735152486, Math.toRadians(0))); // Line 10
        pathPoses.add(new Pose2d(-62.86998394863563, -36.173354735152486, Math.toRadians(0))); // Line 11
        pathPoses.add(new Pose2d(-13.607898448519048, 12.490832157968967, Math.toRadians(20))); // Line 12
        pathPoses.add(new Pose2d(-23.96614950634697, 0.7108603667136748, Math.toRadians(0))); // Line 13

        if (ALLIANCE_COLOR.equals(AllianceColor.RED)) {
            for (Pose2d pose : pathPoses) {
                pose.mirror();
            }
        }
    }

    @Override
    public void initialize() {
        generatePath();

        // Must have for all opModes
        OP_MODE_TYPE = OpModeType.AUTO;

        // Resets the command scheduler
        super.reset();

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);

        robot.launcher.setHood(MAX_HOOD_ANGLE);
        robot.launcher.setRamp(false);
        robot.intake.setPivot(Intake.PivotState.HOLD);
        robot.turret.setTurret(ANGLE_CONTROL, MAX_TURRET_ANGLE * ALLIANCE_COLOR.getMultiplier());

        // Schedule the full auto
        // TODO: FIGURE OUT WHY WE NEED A BURNER INSTANT COMMAND
        schedule(
                new SequentialCommandGroup(
                        // init
                        new InstantCommand(),
                        new InstantCommand(() -> robot.turret.setTurret(ANGLE_CONTROL, MAX_TURRET_ANGLE * ALLIANCE_COLOR.getMultiplier())),
                        new InstantCommand(() -> robot.drive.setPose(pathPoses.get(0))),

                        // preload
                        pathShoot(1, 3500),

                        // spike 1
                        pathIntake(2, 1867, 0.50),
                        new DriveTo(pathPoses.get(4)).withTimeout(1000), // gate
                        pathShoot(5, 2250),

                        // spike 2
                        new DriveTo(pathPoses.get(6)).withTimeout(1000),
                        pathIntake(7, 2267, 0.50),
                        pathShoot(9, 3000),

                        // spike 3
                        new DriveTo(pathPoses.get(10)).withTimeout(2267),
                        pathIntake(11, 2267, 0.50),
                        pathShoot(12, 2250, false),
                        
                        new DriveTo(pathPoses.get(13)), // park
                        new InstantCommand(() -> END_POSE = robot.drive.getPose())
                )
        );
    }

    @Override
    public void initialize_loop() {
        telemetryData.addData("ALLIANCE_COLOR", ALLIANCE_COLOR);
        telemetryData.update();
    }

    @Override
    public void run() {
        // DO NOT REMOVE! Runs the command scheduler and updates telemetry
        robot.updateLoop(telemetryData);

        if (timer == null) {
            timer = new ElapsedTime();
        }

        // Update any constants that are being updated by FTCDash - used for tuning
        for (CoaxialSwerveModule module : robot.drive.swerve.getModules()) {
            module.setSwervoPIDF(SWERVO_PIDF_COEFFICIENTS);
        }
        ((PIDFController) robot.drive.follower.xController).setCoefficients(XY_COEFFICIENTS);
        ((PIDFController) robot.drive.follower.yController).setCoefficients(XY_COEFFICIENTS);
        ((PIDFController) robot.drive.follower.headingController).setCoefficients(HEADING_COEFFICIENTS);

        telemetryData.addData("Loop Time", timer.milliseconds());
        timer.reset();

        if (PROBLEMATIC_TELEMETRY) {
            robot.profiler.start("High TelemetryData");

            telemetryData.addData("Heading", robot.drive.getPose().getHeading());
            telemetryData.addData("Robot Pose", robot.drive.getPose());
            telemetryData.addData("Turret Position", robot.turret.getPosition());
            telemetryData.addData("Flywheel Velocity", robot.launchEncoder.getCorrectedVelocity());
            telemetryData.addData("Intake overCurrent", ((MotorEx) robot.intakeMotors.getMotor()).isOverCurrent());
            telemetryData.addData("FR Module", robot.drive.swerve.getModules()[0].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[0].getPowerTelemetry());
            telemetryData.addData("FL Module", robot.drive.swerve.getModules()[1].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[1].getPowerTelemetry());
            telemetryData.addData("BL Module", robot.drive.swerve.getModules()[2].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[2].getPowerTelemetry());
            telemetryData.addData("BR Module", robot.drive.swerve.getModules()[3].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[3].getPowerTelemetry());

            robot.profiler.end("High TelemetryData");
        }

        robot.profiler.start("Low TelemetryData");
        telemetryData.addData("Robot Target", robot.drive.follower.getTarget());
        telemetryData.addData("atTarget", robot.drive.follower.atTarget());
        telemetryData.addData("X Error", robot.drive.follower.getError().getTranslation().getX());
        telemetryData.addData("Y Error", robot.drive.follower.getError().getTranslation().getY());
        telemetryData.addData("Heading Error", robot.drive.follower.getError().getRotation().getAngle(AngleUnit.RADIANS));

        telemetryData.addData("Turret State", Turret.turretState);
        telemetryData.addData("Turret Target", robot.turret.getTarget());
        telemetryData.addData("Turret readyToLaunch", robot.turret.readyToLaunch());

        telemetryData.addData("Flywheel Active Control", robot.launcher.getActiveControl());
        telemetryData.addData("Flywheel Target Ball Velocity", robot.launcher.getTargetFlywheelVelocity());
        telemetryData.addData("Flywheel Target", robot.launcher.getFlywheelTarget());

        telemetryData.addData("Intake Motor State", Intake.motorState);
        telemetryData.addData("Intake Jammed", robot.intake.intakeJammed);

        telemetryData.addData("Target Chassis Velocity", robot.drive.swerve.getTargetVelocity());
    }

    @Override
    public void end() {
        END_POSE = robot.drive.getPose();
    }

    public SequentialCommandGroup pathShoot(int pathStartingIndex, long timeout) {
        return pathShoot(pathStartingIndex, timeout, true);
    }

    public SequentialCommandGroup pathShoot(int pathStartingIndex, long timeout, boolean pathShoot) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new ConditionalCommand(
                                new DriveTo(pathPoses.get(pathStartingIndex)).withTimeout(timeout),
                                new InstantCommand(),
                                () -> pathShoot
                        ),
                        new InstantCommand(() -> robot.turret.setTurret(GOAL_LOCK_CONTROL, MAX_TURRET_ANGLE * ALLIANCE_COLOR.getMultiplier())),
                        new InstantCommand(() -> robot.launcher.setFlywheel(LAUNCHER_FAR_VELOCITY, true))
                ),
//                new FullAim().withTimeout(2000),
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
                new SetIntake(Intake.MotorState.FORWARD, Intake.PivotState.FORWARD),
                new DriveTo(pathPoses.get(pathStartingIndex), maxPower).withTimeout(timeout),
                new SetIntake(Intake.MotorState.STOP, Intake.PivotState.HOLD),
                new DriveTo(pathPoses.get(pathStartingIndex+1)).withTimeout(2467)
        );
    }
}