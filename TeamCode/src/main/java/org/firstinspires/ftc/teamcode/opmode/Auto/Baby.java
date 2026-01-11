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
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Robot;
import java.util.ArrayList;

@Config
@Autonomous(name = "Baby (far 12+ Ball)", preselectTeleOp = "AAAFullTeleOp")
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

        pathPoses.add(new Pose2d(-15.182108626198076, -64.63897763578274, Math.toRadians(90))); // Starting Pose
        pathPoses.add(new Pose2d(-39.8890839945299, -42.30155979202772, Math.toRadians(-65))); // Line 1
        pathPoses.add(new Pose2d(-49.87175297199957, -42.30155979202772, Math.toRadians(-65))); // Line 2
        pathPoses.add(new Pose2d(-13.090909090909083, -53.49216300940439, Math.toRadians(15))); // Line 3
        pathPoses.add(new Pose2d(-62.068965517241374, -62.746081504702204, Math.toRadians(15))); // Line 4
        pathPoses.add(new Pose2d(-12.865203761755485, -53.71786833855799, Math.toRadians(15))); // Line 5 (Same as 7)
        pathPoses.add(new Pose2d(-61.850955744963166, -55.02946273830156, Math.toRadians(0))); // Line 6
        pathPoses.add(new Pose2d(-12.935877755361787, -53.78162911611784, Math.toRadians(0))); // Line 7 (Same as 5)
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

        // Must have for all opModes
        OP_MODE_TYPE = OpModeType.AUTO;

        // Resets the command scheduler
        super.reset();

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);

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
                        new StationaryAimbotFullLaunch(),

                        // Spike 1 Sequence
                        new SetIntake(Intake.MotorState.FORWARD),
                        new DriveTo(pathPoses.get(1)).withTimeout(1367),
                        new DriveTo(pathPoses.get(2)).withTimeout(800),
                        new SetIntake(Intake.MotorState.STOP),
                        pathShoot(3, 1200),

                        // Balls on wall Sequence
                        instantPathIntake(4, 1200),
                        pathShoot(5, 1300),

                        // Repeatedly intake + shoot balls
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        new ConditionalCommand(
                                                instantPathIntake(6, 1300),
                                                instantPathIntake(6, 1300),
                                                () -> togglePath
                                        ),
                                        new InstantCommand(() -> togglePath = !togglePath),
                                        pathShoot(5, 1300)
                                ),
                                REPEAT_INTAKE
                        ),

                        // Park
                        new DriveTo(pathPoses.get(8)).alongWith(
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

                new StationaryAimbotFullLaunch()
        );
    }

    public SequentialCommandGroup pathIntake(int pathStartingIndex, long timeout) {
        return new SequentialCommandGroup(
                new DriveTo(pathPoses.get(pathStartingIndex)).withTimeout(timeout),
                new ParallelCommandGroup(
                        new SetIntake(Intake.MotorState.FORWARD),
                        new DriveTo(pathPoses.get(pathStartingIndex + 1)).withTimeout(1367)
                ),
                new SetIntake(Intake.MotorState.STOP)
        );
    }

    public SequentialCommandGroup instantPathIntake(int pathStartingIndex, long timeout) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SetIntake(Intake.MotorState.FORWARD),
                        new DriveTo(pathPoses.get(pathStartingIndex)).withTimeout(timeout)
                ),
                new SetIntake(Intake.MotorState.STOP)
        );
    }
}