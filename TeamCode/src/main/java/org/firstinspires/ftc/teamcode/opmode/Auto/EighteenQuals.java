package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.ANGLE_CONTROL;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.GOAL_LOCK_CONTROL;
import static org.firstinspires.ftc.teamcode.globals.Constants.*;
import static org.firstinspires.ftc.teamcode.globals.Constants.AutoConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
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
@Autonomous(name = "EighteenQuals", preselectTeleOp = "FullTeleOp", group = "Auto")
public class EighteenQuals extends CommandOpMode {
    public ElapsedTime timer;

    public ElapsedTime autoTimer;
    public static int REPEAT_TIMES = 3;

    public static boolean GATE_OPEN = true;
    TelemetryEx telemetryEx = new TelemetryEx(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();
    public ArrayList<Pose2d> pathPoses;

    public void generatePath() {
        pathPoses = new ArrayList<>();

        pathPoses.add(new Pose2d(-40.42101920131037, 54.2016782054414254, Math.toRadians(0))); // Starting Pose
        pathPoses.add(new Pose2d(-16.90512605092437, 12.252016806134446, Math.toRadians(5))); // Line 1
        pathPoses.add(new Pose2d(-59.0090756307563, 12.453697478403353, Math.toRadians(0))); // Line 2
        pathPoses.add(new Pose2d(-39.70710940350711, -0.6466988170694802, Math.toRadians(0))); // Line 3
        pathPoses.add(new Pose2d(-55.98075630302521, -0.6466988170694802, Math.toRadians(0))); // Line 4
        pathPoses.add(new Pose2d(-18.095734995924172, 12.459233342037919, Math.toRadians(5))); // Line 5
        pathPoses.add(new Pose2d(-27.165630252605048, -13.896302521596644, Math.toRadians(0))); // Line 6
        pathPoses.add(new Pose2d(-56.80260504252101, -7.421092437563023, Math.toRadians(0))); // Line 7
        pathPoses.add(new Pose2d(-12.039579832436978, 5.604957982605048, Math.toRadians(5))); // Line 8
        pathPoses.add(GATE_FIRST_POSE.get()); // Line 9
        pathPoses.add(GATE_SECOND_POSE.get()); // Line 10
        pathPoses.add(new Pose2d(-13.421800947867297, 3.9810426540284354, Math.toRadians(5))); // Line 11
        pathPoses.add(new Pose2d(-12.38388625592417, 39.01421800947868, Math.toRadians(45))); // Line 12

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
        robot.drive.follower.setTarget(pathPoses.get(1));

        robot.turret.setTurret(ANGLE_CONTROL, (3 * Math.PI) / 4 * ALLIANCE_COLOR.getMultiplier());
        robot.turret.setTurret(GOAL_LOCK_CONTROL, 0);
        robot.readyToLaunch = true;

        Launcher.DISTANCE_OFFSET = -0.241;

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

                        // intake 2nd spike
                        new DriveTo(pathPoses.get(6)).withTimeout(867),
                        pathIntake(7,1000, 0.45),
                        new WaitCommand(1250),

                        // shoot 2nd spike
                        pathShoot(8, 1200),

                        // gate intake cycles
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        // Intake turns on, drives to 9 + 10, turns off upon arrival
                                        gateIntake(9, 2500),

                                        // Drives to 11 and shoots
                                        pathShoot(11, 1300)
                                ),
                                Math.max(1, REPEAT_TIMES)
                        ),

                        // intake 1st spike
                        new SetIntake(Intake.MotorState.FORWARD),
                        new DriveTo(pathPoses.get(2), 0.6)
                                .withTimeout(1467)
                                .interruptOn(() -> robot.intake.transferFull()),

                        // shoot 1st spike
                        pathShoot(12, 1600, false),

                        // park + end
                        new InstantCommand(() -> robot.turret.setTurretPos(0.5, true)),
                        new InstantCommand(() -> robot.intake.setIntake(Intake.MotorState.STOP))
//                        new DriveTo(pathPoses.get(pathPoses.size() - 1))
                )
        );
    }

    @Override
    public void initialize_loop() {
        if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
            REPEAT_TIMES++;
        } else if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
            REPEAT_TIMES--;
            REPEAT_TIMES = Math.max(1, REPEAT_TIMES);
        }

        if (gamepad1.right_bumper) {
            robot.drive.swerve.updateWithTargetVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            robot.drive.follower.calculate(robot.drive.getPose()),
                            robot.drive.getPose().getRotation()
                    ).scale(0.000001)
            );
        } else {
            robot.drive.swerve.stop();
        }

        telemetryEx.addData("REPEAT_TIMES", REPEAT_TIMES);
        robot.initializeLoop(gamepad1, telemetryEx);
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
            autoTimer = new ElapsedTime();
        }

        if (PROBLEMATIC_TELEMETRY) {
            robot.profiler.start("TelemetryData");

            // Always log Loop Time
            telemetryEx.addData("Loop Time", timer.milliseconds());
            timer.reset();

            telemetryEx.addData("Robot Pose", robot.drive.getPose());
            telemetryEx.addData("Robot Target", robot.drive.follower.getTarget());
            telemetryEx.addData("atTarget", robot.drive.follower.atTarget());
            telemetryEx.addData("Heading", robot.drive.getPose().getHeading());
            telemetryEx.addData("Heading Coefficients", Arrays.toString(((PIDFController)robot.drive.follower.headingController).getCoefficients()));
            telemetryEx.addData("Target Chassis Velocity", robot.drive.swerve.getTargetVelocity());

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
        robot.updateLoop(null);
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
        return pathShoot(pathStartingIndex, timeout, true);
    }

    public SequentialCommandGroup pathShoot(int pathStartingIndex, long timeout, boolean stopInZone) {
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        new DriveTo(pathPoses.get(pathStartingIndex), 0.7).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new SetIntake(Intake.MotorState.STOP)
                                )
                        ),
                        new WaitCommand(timeout),
                        new ConditionalCommand(
                                new WaitUntilCommand(() -> Drive.robotInZone(robot.drive.getPose())),
                                new RunCommand(() -> {}),
                                () -> stopInZone
                        )

                ),
                new ParallelRaceGroup(
                        new RunCommand(() -> robot.drive.swerve.updateWithXLock()),
                        new ClearLaunch().beforeStarting(new WaitCommand(100))
                )
        );
    }


    public SequentialCommandGroup gateIntake(int pathStartingIndex, long timeout) {
        return new SequentialCommandGroup(
                new DriveTo(pathPoses.get(pathStartingIndex), 0.0, 0.67).withTimeout(1400),
                new SequentialCommandGroup(
                        new SetIntake(Intake.MotorState.FORWARD),
                        new DriveTo(pathPoses.get(pathStartingIndex + 1), 0.0, 0.5, 0)
                                .withTimeout(timeout)
                                .alongWith(new WaitCommand(timeout))
                ).interruptOn(() -> robot.intake.transferFull()),

                new ConditionalCommand(
                        new WaitCommand(250),
                        new InstantCommand(),
                        () -> robot.intake.transferFull()
                ),

                new InstantCommand(() -> robot.launcher.setTransfer(false))
        );
    }

    public SequentialCommandGroup pathIntake(int pathStartingIndex, long timeout) {
        return pathIntake(pathStartingIndex, timeout, 0.6);
    }

    public SequentialCommandGroup pathIntake(int pathStartingIndex, long timeout, double maxPower) {
        return new SequentialCommandGroup(
                new SetIntake(Intake.MotorState.FORWARD),
                new DriveTo(pathPoses.get(pathStartingIndex), maxPower).withTimeout(timeout),
                new SetIntake(Intake.MotorState.STOP)
        );
    }
}