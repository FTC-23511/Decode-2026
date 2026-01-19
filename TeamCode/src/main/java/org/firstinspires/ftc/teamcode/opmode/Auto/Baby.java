package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.GOAL_LOCK_CONTROL;
import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.*;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commandbase.commands.*;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Robot;
import java.util.ArrayList;

@Config
@Autonomous(name = "Baby (far 12+ Ball)", preselectTeleOp = "AAAFullTeleOp", group = "Auto")
public class Baby extends CommandOpMode {
    public ElapsedTime timer;
    public static boolean GATE_OPEN = false;
    TelemetryData telemetryData = new TelemetryData(
            new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())
    );
    private final Robot robot = Robot.getInstance();
    public ArrayList<Pose2d> pathPoses;

    public static int REPEAT_INTAKE = 3;
    private boolean togglePath = true;

    public void generatePath() {
        pathPoses = new ArrayList<>();

        pathPoses.add(new Pose2d(-15.25, -64.5, Math.toRadians(0))); // Starting Pose
        pathPoses.add(new Pose2d(-16.332268370607025, -58.58785942492013, Math.toRadians(0))); // Line 1
        pathPoses.add(new Pose2d(-22.08306709265176, -37.035143769968045, Math.toRadians(0))); // Line 2
        pathPoses.add(new Pose2d(-64.10862619808307, -35.884984025559106, Math.toRadians(0))); // Line 3
        pathPoses.add(new Pose2d(-16.332268370607025, -58.58785942492013, Math.toRadians(0))); // Line 4
        pathPoses.add(new Pose2d(-66.327447833065804, -53.97110754414126, Math.toRadians(15))); // Line 5
        pathPoses.add(new Pose2d(-66.327447833065804, -62.746081504702204, Math.toRadians(15))); // Line 6
        pathPoses.add(new Pose2d(-16.332268370607025, -58.58785942492013, Math.toRadians(0))); // Line 7
        pathPoses.add(new Pose2d(-66.798722044728436, -53.82747603833866, Math.toRadians(0))); // Line 8
        pathPoses.add(new Pose2d(-20.08777429467085, -33.40438871473354, Math.toRadians(0))); // Line 9

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
        TESTING_OP_MODE = false;

        // Resets the command scheduler
        super.reset();

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);

        Launcher.DISTANCE_OFFSET = 0.15;

        robot.launcher.setHood(MAX_HOOD_ANGLE);
        robot.launcher.setRamp(false);
        robot.turret.setTurret(GOAL_LOCK_CONTROL, 0);

        // Schedule the full auto
        schedule(
                new SequentialCommandGroup(
                        // Set starting pose
                        new InstantCommand(),
                        new InstantCommand(() -> robot.drive.setPose(pathPoses.get(0))),

                        // Score Preload
                        pathShoot(1, 750),

                        // Spike 1 Sequence
                        pathIntake(2, 1000),
                        pathShoot(4, 1200),

                        // Balls on wall Sequence
                        new SetIntake(Intake.MotorState.FORWARD),
                        new DriveTo(pathPoses.get(5)).withTimeout(1300),
                        new DriveTo(pathPoses.get(5)).withTimeout(750),
                        new SetIntake(Intake.MotorState.STOP),
                        pathShoot(7, 1300),

                        // Repeatedly intake + shoot balls
                        new SequentialCommandGroup(
                                instantPathIntake(8, 1600),
                                pathShoot(7, 1300)
                        ),

                        new SequentialCommandGroup(
                                instantPathIntake(8, 1600),
                                pathShoot(7, 1300)
                        ),

                        // Park
                        new DriveTo(pathPoses.get(9)).alongWith(
                                new InstantCommand(() -> robot.launcher.setFlywheel(0, false))
                        )
                )
        );
    }

    @Override
    public void initialize_loop() {
        if (gamepad1.cross || gamepad2.cross || gamepad1.triangle || gamepad2.triangle) {
            GATE_OPEN = false;
        } else if (gamepad1.circle || gamepad2.circle || gamepad1.square || gamepad2.square) {
            GATE_OPEN = true;
        }

        telemetryData.addData("Gate Open", GATE_OPEN);
        telemetryData.addData("Alliance Color", ALLIANCE_COLOR);
        telemetryData.update();
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
        }

        // Always log Loop Time
        telemetryData.addData("Loop Time", timer.milliseconds());

        timer.reset();

        if (PROBLEMATIC_TELEMETRY) {
            robot.profiler.start("TelemetryData");

            telemetryData.addData("Robot Pose", robot.drive.getPose());
            telemetryData.addData("Robot Target", robot.drive.follower.getTarget());
            telemetryData.addData("atTarget", robot.drive.follower.atTarget());
            telemetryData.addData("Heading", robot.drive.getPose().getHeading());

            telemetryData.addData("Turret State", Turret.turretState);
            telemetryData.addData("Turret Target", robot.turret.getTarget());
            telemetryData.addData("Turret readyToLaunch", robot.turret.readyToLaunch());
            telemetryData.addData("Turret Position", robot.turret.getPosition());

            telemetryData.addData("Flywheel Velocity", robot.launchEncoder.getCorrectedVelocity());
            telemetryData.addData("Flywheel Active Control", robot.launcher.getActiveControl());
            telemetryData.addData("Flywheel Target Ball Velocity", robot.launcher.getTargetFlywheelVelocity());
            telemetryData.addData("Flywheel Target", robot.launcher.getFlywheelTarget());
            telemetryData.addData("Flywheel Ready", robot.launcher.flywheelReady());

            telemetryData.addData("Target Chassis Velocity", robot.drive.swerve.getTargetVelocity());

            robot.profiler.end("TelemetryData");
        }

        robot.profiler.start("Run + Update");
        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        robot.updateLoop(telemetryData);
        robot.profiler.end("Run + Update");

        robot.profiler.end("Full Loop");
    }

    @Override
    public void end() {
        END_POSE = robot.drive.getPose();
    }

    public SequentialCommandGroup pathShoot(int pathStartingIndex, long timeout) {
        return new SequentialCommandGroup(
                new DriveTo(pathPoses.get(pathStartingIndex)).withTimeout(timeout).alongWith(
                        new InstantCommand(() -> robot.launcher.setLauncher(pathPoses.get(pathStartingIndex)))
                ),
                new FullAim().withTimeout(1000),
                new WaitCommand(250),
                new ClearLaunch(true).alongWith(
                        new InstantCommand(
                                () -> robot.drive.swerve.updateWithXLock() // Lock the drivetrain wheel to an X shape reduce how much we can be pushed)
                        )
                ),
                new WaitCommand(300)
        );
    }

    public SequentialCommandGroup pathIntake(int pathStartingIndex, long timeout) {
        return new SequentialCommandGroup(
                new DriveTo(pathPoses.get(pathStartingIndex)).withTimeout(timeout),
                new InstantCommand(() -> robot.launcher.setRamp(false)),
                new ParallelCommandGroup(
                        new SetIntake(Intake.MotorState.FORWARD),
                        new DriveTo(pathPoses.get(pathStartingIndex + 1), 0.67).withTimeout(1500)
                ),
                new SetIntake(Intake.MotorState.STOP)
        );
    }

    public SequentialCommandGroup instantPathIntake(int pathStartingIndex, long timeout) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> robot.launcher.setRamp(false)),
                new ParallelCommandGroup(
                        new SetIntake(Intake.MotorState.FORWARD),
                        new DriveTo(pathPoses.get(pathStartingIndex), 0.5).withTimeout(timeout)
                ),
                new SetIntake(Intake.MotorState.STOP)
        );
    }
}