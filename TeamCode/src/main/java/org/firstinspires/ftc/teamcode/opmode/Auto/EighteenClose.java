package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.ANGLE_CONTROL;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.GOAL_LOCK_CONTROL;
import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.TelemetryEx;

import org.firstinspires.ftc.teamcode.commandbase.commands.ClearLaunch;
import org.firstinspires.ftc.teamcode.commandbase.commands.DriveTo;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.ArrayList;
import java.util.Arrays;

@Config
@Autonomous(name = "EighteenClose", preselectTeleOp = "FullTeleOp", group = "Auto")
public class EighteenClose extends CommandOpMode {
    public ElapsedTime timer;

    public ElapsedTime autoTimer;
    public static int REPEAT_TIMES = 3;

    public static boolean GATE_OPEN = true;
    TelemetryEx telemetryEx = new TelemetryEx(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();
    public ArrayList<Pose2d> pathPoses;

    public void generatePath() {
        pathPoses = new ArrayList<>();

        pathPoses.add(new Pose2d(-45.3781512605042,   55.05882352941177, Math.toRadians(0))); // Starting Pose
        pathPoses.add(new Pose2d(-20.19327731092437,  12.100840336134446, Math.toRadians(5))); // Line 1
        pathPoses.add(new Pose2d(-60.0672268907563,   12.302521008403353, Math.toRadians(0))); // Line 2
        pathPoses.add(new Pose2d(-46.58823529411764,  2.50168067226890685, Math.toRadians(0))); // Line 3
        pathPoses.add(new Pose2d(-59.26890756302521,  0.20212471293051984, Math.toRadians(0))); // Line 4
        pathPoses.add(new Pose2d(-18.352941176470583, 8.26890756302521182, Math.toRadians(5))); // Line 5
        pathPoses.add(new Pose2d(-30.453781512605048, -13.697478991596643, Math.toRadians(0))); // Line 6
        pathPoses.add(new Pose2d(-60.090756302521005, -6.572268907563023, Math.toRadians(0))); // Line 7
        pathPoses.add(new Pose2d(-15.327731092436977, 6.4537815126050475, Math.toRadians(5))); // Line 8
        pathPoses.add(new Pose2d(-59.00025210084033, -18.012605042016805, Math.toRadians(-25))); // Line 9
        pathPoses.add(new Pose2d(-60.54803149620063, -7.637795275590548, Math.toRadians(-25))); // Line 10
        pathPoses.add(new Pose2d(-15.327731092436977, 6.4537815126050475, Math.toRadians(5))); // Line 11
        pathPoses.add(new Pose2d(-28.55462184873949, -3.6302521008403374, Math.toRadians(0))); // Line 12

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

        robot.launcher.setHood(MAX_HOOD_ANGLE);
        robot.launcher.setRamp(false);

        robot.drive.setPose(pathPoses.get(0));
        robot.turret.setTurret(ANGLE_CONTROL, (3 * Math.PI) / 4 * ALLIANCE_COLOR.getMultiplier());
        robot.turret.setTurret(GOAL_LOCK_CONTROL, 0);
        robot.readyToLaunch = true;

        Launcher.DISTANCE_OFFSET = -0.167;

        // Schedule the full auto
        schedule(
                new SequentialCommandGroup(
                        // Set starting pose
                        new InstantCommand(),
                        new InstantCommand(() -> robot.drive.setPose(pathPoses.get(0))),
                        new InstantCommand(() -> robot.drive.swerve.setMaxSpeed(0.6)),
                        new InstantCommand(() -> robot.readyToLaunch = true),

                        // preload
                        new DriveTo(pathPoses.get(1), 0.767).withTimeout(1425),
                        new ParallelRaceGroup(
                                new RunCommand(() -> robot.drive.swerve.updateWithXLock()),
                                new ClearLaunch().beforeStarting(new WaitCommand(100))
                        ),

                        // intake 1st spike
                        pathIntake(2, 1250),

                        // open gate
                        new DriveTo(pathPoses.get(3), 1).withTimeout(400),
                        new DriveTo(pathPoses.get(4)).withTimeout(967),
                        new WaitCommand(500),

                        // shoot 1st spike
                        pathShoot(5, 1450),

                        // intake 2nd spike
                        new DriveTo(pathPoses.get(6)).withTimeout(867),
                        pathIntake(7,1350),
                        new WaitCommand(500),

                        // shoot 2nd spike
                        pathShoot(8, 1550),

                        // gate intake cycles
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        // Intake turns on, drives to 9 + 10, turns off upon arrival
                                        gateIntake(9, 2000),

                                        // Drives to 11 and shoots
                                        pathShoot(11, 1550)
                                ),
                                REPEAT_TIMES
                        ),

                        // park + end
                        new InstantCommand(() -> robot.turret.setTurretPos(0.5, true)),
                        new InstantCommand(() -> robot.intake.setIntake(Intake.MotorState.STOP)),
                        new DriveTo(pathPoses.get(pathPoses.size() - 1))
                )
        );
    }

    @Override
    public void initialize_loop() {
        if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
            REPEAT_TIMES++;
        } else if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
            REPEAT_TIMES--;
            REPEAT_TIMES = Math.max(0,REPEAT_TIMES);
        }

        telemetryEx.addData("REPEAT_TIMES", REPEAT_TIMES);
        telemetryEx.addData("GATE_OPEN", GATE_OPEN);
        robot.initializeLoop(gamepad1, telemetryEx);
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
            autoTimer = new ElapsedTime();
        }

        // Always log Loop Time
        telemetryEx.addData("Loop Time", timer.milliseconds());

        timer.reset();

        if (PROBLEMATIC_TELEMETRY) {
            robot.profiler.start("TelemetryData");
//
            telemetryEx.addData("Robot Pose", robot.drive.getPose());
            telemetryEx.addData("Robot Target", robot.drive.follower.getTarget());
            telemetryEx.addData("atTarget", robot.drive.follower.atTarget());
            telemetryEx.addData("Heading", robot.drive.getPose().getHeading());
            telemetryEx.addData("Heading Coefficients", Arrays.toString(((PIDFController)robot.drive.follower.headingController).getCoefficients()));
            telemetryEx.addData("Target Chassis Velocity", robot.drive.swerve.getTargetVelocity());
//
            telemetryEx.addData("Turret State", Turret.turretState);
            telemetryEx.addData("Turret Target", robot.turret.getTarget());
            telemetryEx.addData("Turret readyToLaunch", robot.turret.readyToLaunch());
            telemetryEx.addData("Turret Position", robot.turret.getRelativePos());
            telemetryEx.update();

//            telemetryData.addData("Target Chassis Velocity", robot.drive.swerve.getTargetVelocity());

            robot.profiler.end("TelemetryData");
        }

        robot.profiler.start("Run + Update");
        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        robot.updateLoop(telemetryEx);
        robot.profiler.end("Run + Update");

        robot.profiler.end("Full Loop");
    }

    @Override
    public void end() {
        END_POSE = robot.drive.getPose();
        robot.turret.setTurretPos(0.5, true);
        robot.intake.setIntake(Intake.MotorState.STOP);
        telemetryEx.update();
        robot.exportProfiler(robot.profilerFile, robot.logCatFile);
    }

    public SequentialCommandGroup pathShoot(int pathStartingIndex, long timeout) {
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        new DriveTo(pathPoses.get(pathStartingIndex), 0.7),
                        new WaitCommand(timeout),
                        new WaitUntilCommand(() -> Drive.robotInZone(robot.drive.getPose())).andThen(new WaitCommand(100))
                ),
                new ParallelRaceGroup(
                        new RunCommand(() -> robot.drive.swerve.updateWithXLock()),
                        new ClearLaunch().beforeStarting(new WaitCommand(100))
                )
        );
    }

    public SequentialCommandGroup gateIntake(int pathStartingIndex, long timeout) {
        return new SequentialCommandGroup(
                new DriveTo(pathPoses.get(pathStartingIndex), 0.67).withTimeout(1200),
                new SetIntake(Intake.MotorState.FORWARD),
                new DriveTo(pathPoses.get(pathStartingIndex + 1), AUTO_MIN_POWER, 0.5, 0)
                        .withTimeout(timeout)
                        .alongWith(new WaitCommand(timeout)),
                new SetIntake(Intake.MotorState.STOP),
                new InstantCommand(() -> robot.launcher.setTransfer(false))
        );
    }

    public SequentialCommandGroup pathIntake(int pathStartingIndex, long timeout) {
        return new SequentialCommandGroup(
                new SetIntake(Intake.MotorState.FORWARD),
                new DriveTo(pathPoses.get(pathStartingIndex), 0.6).withTimeout(timeout),
                new SetIntake(Intake.MotorState.STOP)
        );
    }
}