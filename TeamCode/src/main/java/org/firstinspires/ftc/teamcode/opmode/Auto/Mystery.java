package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.ANGLE_CONTROL;
import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.swerve.coaxial.CoaxialSwerveModule;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commandbase.commands.ClearLaunch;
import org.firstinspires.ftc.teamcode.commandbase.commands.DriveTo;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.ArrayList;

@Config
@Autonomous(name = "Mystery (far auto)", preselectTeleOp = "AAAFullTeleOp")
public class Mystery extends CommandOpMode {
    public ElapsedTime timer;

    TelemetryData telemetryData = new TelemetryData(
            new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())
    );

    private final Robot robot = Robot.getInstance();

    public ArrayList<Pose2d> pathPoses;
    public void generatePath() {
        pathPoses = new ArrayList<>();

        pathPoses.add(new Pose2d(-17.11568938193344, -64.93, Math.toRadians(90))); // Starting Pose
        pathPoses.add(new Pose2d(-16.204, 27.038, Math.toRadians(143))); // Line 1
        pathPoses.add(new Pose2d(-26, 11.5, Math.toRadians(0))); // Line 2
        pathPoses.add(new Pose2d(-50.59587955625991, 11.5, Math.toRadians(0))); // Line 3
        pathPoses.add(new Pose2d(-16.202, 27.038, Math.toRadians(143.001))); // Line 4
        pathPoses.add(new Pose2d(-24.190174326465925, -11.296354992076065, Math.toRadians(0))); // Line 5
        pathPoses.add(new Pose2d(-54.997, -11.296354992076065, Math.toRadians(0))); // Line 6
        pathPoses.add(new Pose2d(-31.492868462757528, -11.296354992076065, Math.toRadians(0))); // Line 7
        pathPoses.add(new Pose2d(-16.202852614896994, 27.042, Math.toRadians(143))); // Line 8
        pathPoses.add(new Pose2d(-31.492868462757528, -11.296354992076065, Math.toRadians(0))); // Line 9

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

        robot.launcher.setHood(MIN_HOOD_SERVO_POS);
        robot.launcher.setRamp(true);
        robot.intake.setPivot(Intake.PivotState.HOLD);
        robot.turret.setTurret(ANGLE_CONTROL, 0);

        // Schedule the full auto
        // TODO: FIGURE OUT WHY WE NEED A BURNER INSTANT COMMAND
        schedule(
                new SequentialCommandGroup(
                        // init
                        new InstantCommand(),
                        new InstantCommand(() -> robot.turret.setTurret(Turret.TurretState.OFF, 0)),
                        new InstantCommand(() -> robot.drive.setPose(pathPoses.get(0))),

                        // preload
                        pathShoot(1, 2500),

                        // spike 1
                        new DriveTo(pathPoses.get(2)).withTimeout(670),
                        pathIntake(3, 1867),
                        pathShoot(5, 2250),

                        // spike 2
                        pathIntake(6, 2267),
                        new DriveTo(pathPoses.get(8)).withTimeout(670),
                        pathShoot(9, 3000),
                        new ClearLaunch(true),

                        new DriveTo(pathPoses.get(10)) // park
                )
        );
    }

    @Override
    public void run() {
        // DO NOT REMOVE
        super.run();
        telemetryData.update();

        // Update any constants that are being updated by FTCDash - used for tuning
        for (CoaxialSwerveModule module : robot.drive.swerve.getModules()) {
            module.setSwervoPIDF(SWERVO_PIDF_COEFFICIENTS);
        }
        ((PIDFController) robot.drive.follower.xController).setCoefficients(XY_COEFFICIENTS);
        ((PIDFController) robot.drive.follower.yController).setCoefficients(XY_COEFFICIENTS);
        ((PIDFController) robot.drive.follower.headingController).setCoefficients(HEADING_COEFFICIENTS);

        telemetryData.addData("Loop Time", timer.milliseconds());
        timer.reset();

        telemetryData.addData("Heading", robot.drive.getPose().getHeading());
        telemetryData.addData("Robot Pose", robot.drive.getPose());
        telemetryData.addData("Target Pose", robot.drive.follower.getTarget());
        telemetryData.addData("Calculated", robot.drive.follower.calculate(robot.drive.getPose()));

        telemetryData.addData("X Error", robot.drive.follower.getError().getTranslation().getX());
        telemetryData.addData("Y Error", robot.drive.follower.getError().getTranslation().getY());
        telemetryData.addData("Heading Error", robot.drive.follower.getError().getRotation().getAngle(AngleUnit.RADIANS));

        telemetryData.addData("Target Chassis Velocity", robot.drive.swerve.getTargetVelocity());
        telemetryData.addData("FR Module", robot.drive.swerve.getModules()[0].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[0].getPowerTelemetry());
        telemetryData.addData("FL Module", robot.drive.swerve.getModules()[1].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[1].getPowerTelemetry());
        telemetryData.addData("BL Module", robot.drive.swerve.getModules()[2].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[2].getPowerTelemetry());
        telemetryData.addData("BR Module", robot.drive.swerve.getModules()[3].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[3].getPowerTelemetry());

        telemetryData.addData("Turret State", Turret.turretState);
        telemetryData.addData("Turret Target", robot.turret.getTarget());
        telemetryData.addData("Turret Position", robot.turret.getPosition());
        telemetryData.addData("Turret readyToLaunch", robot.turret.readyToLaunch());
        telemetryData.addData("LLResult Null", robot.turret.llResult == null);

        telemetryData.addData("Flywheel Active Control", robot.launcher.getActiveControl());
        telemetryData.addData("Flywheel Target Ball Velocity", robot.launcher.getTargetFlywheelVelocity());
        telemetryData.addData("Flywheel Target", robot.launcher.getFlywheelTarget());
        telemetryData.addData("Flywheel Velocity", robot.launchEncoder.getCorrectedVelocity());

        telemetryData.addData("Intake overCurrent", robot.intakeMotor.isOverCurrent());
        telemetryData.addData("Intake Motor State", Intake.motorState);
        telemetryData.addData("Intake Jammed", robot.intake.intakeJammed);

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        telemetryData.update();
    }

    @Override
    public void end() {
        END_POSE = robot.drive.getPose();
    }

    public SequentialCommandGroup pathShoot(int pathStartingIndex, long timeout) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SetIntake(Intake.MotorState.FORWARD, Intake.PivotState.HOLD).beforeStarting(new WaitCommand(410)),
                        new DriveTo(pathPoses.get(pathStartingIndex)).withTimeout(timeout),
                        new InstantCommand(() -> robot.launcher.setFlywheel(LAUNCHER_FAR_VELOCITY, true))
                ),
                new InstantCommand(() -> robot.turret.setTurret(Turret.TurretState.OFF, 0)),
                new WaitUntilCommand(() -> robot.turret.readyToLaunch()).withTimeout(500),
                new InstantCommand(() -> robot.readyToLaunch = true),
                new ClearLaunch(true)
        );
    }

    public SequentialCommandGroup pathIntake(int pathStartingIndex, long timeout) {
        return new SequentialCommandGroup(
                new DriveTo(pathPoses.get(pathStartingIndex)).withTimeout(timeout),
                new SetIntake(Intake.MotorState.FORWARD, Intake.PivotState.FORWARD),

                new DriveTo(pathPoses.get(pathStartingIndex+1)).withTimeout(2467)
        );
    }
}