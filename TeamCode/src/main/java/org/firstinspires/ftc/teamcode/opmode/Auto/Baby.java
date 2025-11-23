package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.ANGLE_CONTROL;
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
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.swerve.coaxial.CoaxialSwerveModule;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commandbase.commands.ClearLaunch;
import org.firstinspires.ftc.teamcode.commandbase.commands.DriveTo;
import org.firstinspires.ftc.teamcode.commandbase.commands.FullAim;
import org.firstinspires.ftc.teamcode.commandbase.commands.PrepDriveTo;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.ArrayList;

@Config
@Autonomous(name = "Baby (far gate auto)", preselectTeleOp = "AAAFullTeleOp")
public class Baby extends CommandOpMode {
    public ElapsedTime timer;

    TelemetryData telemetryData = new TelemetryData(
            new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())
    );

    private final Robot robot = Robot.getInstance();

    public ArrayList<Pose2d> pathPoses;
    public void generatePath() {
        pathPoses = new ArrayList<>();

        pathPoses.add(new Pose2d(-14.8, -65, Math.toRadians(0))); // Starting Pose
        pathPoses.add(new Pose2d(-23.96614950634697, -11.475317348377999, Math.toRadians(0))); // Line 1
        pathPoses.add(new Pose2d(-61.74330042313117, -11.475317348377999, Math.toRadians(0))); // Line 2
        pathPoses.add(new Pose2d(-47.72919605077574, -11.475317348377999, Math.toRadians(0))); // Line 3
        pathPoses.add(new Pose2d(-47.72919605077574, -0.7108603667136748, Math.toRadians(0))); // Line 4
        pathPoses.add(new Pose2d(-59.40288924558587, -0.7108603667136748, Math.toRadians(0))); // Line 5
        pathPoses.add(new Pose2d(-13.090909090909083, -23.45839210155148, Math.toRadians(0))); // Line 6
        pathPoses.add(new Pose2d(-14.8, -65, Math.toRadians(0))); // Line 7
        pathPoses.add(new Pose2d(-60.327447833065804, -53.97110754414126, Math.toRadians(15))); // Line 8
        pathPoses.add(new Pose2d(-62.068965517241374, -62.746081504702204, Math.toRadians(15))); // Line 9
        pathPoses.add(new Pose2d(-12.865203761755485, -53.71786833855799, Math.toRadians(0))); // Line 10
        pathPoses.add(new Pose2d(-15.25521669341894, -65.0658105939005, Math.toRadians(0))); // Line 11
        pathPoses.add(new Pose2d(-23.96614950634697, -36.86318758815233, Math.toRadians(0))); // Line 12
        pathPoses.add(new Pose2d(-60.72778561354019, -36.86318758815233, Math.toRadians(0))); // Line 13
        pathPoses.add(new Pose2d(-12.865203761755485, -53.71786833855799, Math.toRadians(0))); // Line 14
        pathPoses.add(new Pose2d(-15.25521669341894, -65.0658105939005, Math.toRadians(0))); // Line 15
        pathPoses.add(new Pose2d(-29.856135401974612, -55.954866008462616, Math.toRadians(0))); // Line 16

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

        // Resets the command scheduler
        super.reset();

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);

        robot.launcher.setHood(MAX_HOOD_SERVO_POS);
        robot.launcher.setRamp(true);
        robot.intake.setPivot(Intake.PivotState.HOLD);
        robot.turret.setTurret(ANGLE_CONTROL, 1.965 * ALLIANCE_COLOR.getMultiplier());

        // Schedule the full auto
        // TODO: FIGURE OUT WHY WE NEED A BURNER INSTANT COMMAND
        schedule(
                new SequentialCommandGroup(
                        // init
                        new InstantCommand(),
                        new InstantCommand(() -> robot.drive.setPose(pathPoses.get(0))),
                        new InstantCommand(() -> robot.turret.setTurret(ANGLE_CONTROL, 1.965 * ALLIANCE_COLOR.getMultiplier())),

                        // preload
                        pathShoot(0, 3500, false),

                        // spike 1
                        new DriveTo(pathPoses.get(1)).withTimeout(2250),
                        pathIntake(2, 1867, 0.35),
                        new DriveTo(pathPoses.get(4)).withTimeout(750),
                        new DriveTo(pathPoses.get(5)).withTimeout(1267),
                        new DriveTo(pathPoses.get(6)).withTimeout(2250),
                        pathShoot(7, 2250),

                        // spike 2
                        pathIntake(8, 2267, 0.75),
                        pathIntake(9, 2267, 0.35),
                        new DriveTo(pathPoses.get(10)).withTimeout(2250),
                        pathShoot(11, 3000),

                        // spike 3
                        new DriveTo(pathPoses.get(12)).withTimeout(2250),
                        pathIntake(13, 2267, 0.35),
                        pathShoot(15, 3000),

                        new DriveTo(pathPoses.get(16)) // park
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

        telemetryData.addData("Sigma", "Polar");

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        telemetryData.update();
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
                        new SetIntake(Intake.MotorState.FORWARD, Intake.PivotState.HOLD).beforeStarting(new WaitCommand(410)),
                        new ConditionalCommand(
                                new DriveTo(pathPoses.get(pathStartingIndex)).withTimeout(timeout),
                                new InstantCommand(),
                                () -> pathShoot
                        ),
                        new InstantCommand(() -> robot.launcher.setFlywheel(LAUNCHER_VERY_FAR_VELOCITY, true))
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
                new DriveTo(pathPoses.get(pathStartingIndex), maxPower).withTimeout(timeout),
                new SetIntake(Intake.MotorState.FORWARD, Intake.PivotState.FORWARD),

                new DriveTo(pathPoses.get(pathStartingIndex+1)).withTimeout(2467)
        );
    }
}