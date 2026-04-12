package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.GOAL_LOCK_CONTROL;
import static org.firstinspires.ftc.teamcode.globals.Constants.ALLIANCE_COLOR;
import static org.firstinspires.ftc.teamcode.globals.Constants.AllianceColor;
import static org.firstinspires.ftc.teamcode.globals.Constants.END_POSE;
import static org.firstinspires.ftc.teamcode.globals.Constants.MAX_HOOD_ANGLE;
import static org.firstinspires.ftc.teamcode.globals.Constants.OP_MODE_TYPE;
import static org.firstinspires.ftc.teamcode.globals.Constants.OpModeType;
import static org.firstinspires.ftc.teamcode.globals.Constants.PROBLEMATIC_TELEMETRY;
import static org.firstinspires.ftc.teamcode.globals.Constants.TESTING_OP_MODE;
import static org.firstinspires.ftc.teamcode.globals.Constants.TURRET_SYNCED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.util.TelemetryEx;

import org.firstinspires.ftc.teamcode.commandbase.commands.ClearLaunch;
import org.firstinspires.ftc.teamcode.commandbase.commands.DriveTo;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.ArrayList;
import java.util.Arrays;

@Config
@Autonomous(name = "Jinu (quals close 15 Ball)", preselectTeleOp = "FullTeleOp", group = "Auto")
public class Jinu extends CommandOpMode {
    public ElapsedTime timer;

    public ElapsedTime autoTimer;
    public static int REPEAT_TIMES = 2;

    public static boolean GATE_OPEN = false;
    TelemetryEx telemetryEx = new TelemetryEx(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();
    public ArrayList<Pose2d> pathPoses;

    public void generatePath() {
        pathPoses = new ArrayList<>();
        pathPoses.add(new Pose2d(-43.24023076923076, 54.670769233076923, Math.toRadians(0))); // Starting Pose
        pathPoses.add(new Pose2d(-19.82608695652174, 12.637681159420286, Math.toRadians(0))); // Line 1
        pathPoses.add(new Pose2d(-26.55072463768115, -13.492753623188406, Math.toRadians(0))); // Line 2
        pathPoses.add(new Pose2d(-66.00420289855072, -13.492753623188406, Math.toRadians(0))); // Line 3
        pathPoses.add(new Pose2d(-33.55072463768116, -13.565217391304344, Math.toRadians(0))); // Line 4
        pathPoses.add(new Pose2d(-15.65217391304348, 9.623188405797105, Math.toRadians(0))); // Line 5
        pathPoses.add(new Pose2d(-36.52173913043478, -15.652173913043484, Math.toRadians(-10))); // Line 6 // weird spline cutoff
        pathPoses.add(new Pose2d(-55.23188405797101, -4.7681159420289845, Math.toRadians(-10))); // Line 7
        pathPoses.add(new Pose2d(-48.00000000000000, -22.000000000000000, Math.toRadians(-45))); // Line 8
        pathPoses.add(new Pose2d(-62.63679999999999, -20.782400000000006, Math.toRadians(-45))); // Line 9
        pathPoses.add(new Pose2d(-35.13043478260869, -10.782608695652172, Math.toRadians(0))); // Line 10
        pathPoses.add(new Pose2d(-23.53623188405797, 12.637681159420286, Math.toRadians(0))); // Line 11
        pathPoses.add(new Pose2d(-55.52173913043479, 12.637681159420286, Math.toRadians(0))); // Line 12
        pathPoses.add(new Pose2d(-23.615999999999993, 12.787199999999999, Math.toRadians(0))); // Line 13
        pathPoses.add(new Pose2d(-49.507246376811594, -10.086956521739125, Math.toRadians(-27.5))); // Line 14
        pathPoses.add(new Pose2d(-56.23188405797101, -7.536231884057969, Math.toRadians(-27.5))); // Line 15
        pathPoses.add(new Pose2d(-58.636799999999994, -15.782400000000006, Math.toRadians(-35))); // Line 16
        pathPoses.add(new Pose2d(-22.84057971014493, 11.710144927536223, Math.toRadians(0))); // Line 17
        pathPoses.add(new Pose2d(-31.42028985507247, 5.217391304347821, Math.toRadians(0))); // Line 18

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
        robot.turret.setTurret(GOAL_LOCK_CONTROL, (3 * Math.PI) / 4 * ALLIANCE_COLOR.getMultiplier());

        // Schedule the full auto
        schedule(
                new SequentialCommandGroup(
                        // Set starting pose
                        new InstantCommand(),
                        new InstantCommand(() -> robot.drive.setPose(pathPoses.get(0))),
                        new InstantCommand(() -> robot.drive.swerve.setMaxSpeed(0.9)),

                        // preload
                        pathShoot(1, 1600),

                        // 2nd spike mark
                        new DriveTo(pathPoses.get(2)).withTimeout(967),
                        pathIntake(3, 1670),

                        new DriveTo(pathPoses.get(4)).withTimeout(600),
                        pathShoot(5, 1250),

                        // gate intake cycles
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        new DriveTo(pathPoses.get(6), 1.0).withTimeout(500),
                                        new DriveTo(pathPoses.get(7), 0.5).withTimeout(800),
                                        new WaitCommand(200),
                                        new DriveTo(pathPoses.get(8), 1.0).withTimeout(467),
                                        new SetIntake(Intake.MotorState.FORWARD),
                                        gateIntake(9, 1267),

                                        new DriveTo(pathPoses.get(10)).withTimeout(900),
                                        pathShoot(11, 1400)
                                ),
                                REPEAT_TIMES
                        ),

                        // 1st spike mark
                        pathIntake(12, 1267),
                        pathShoot(13, 1567),

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

        if (gamepad1.right_stick_button) {
            robot.pinpoint.resetPosAndIMU();
        }

        telemetryEx.addData("Gate Open", GATE_OPEN);
        telemetryEx.addData("TURRET_SYNCED", TURRET_SYNCED);
        telemetryEx.addData("Alliance Color", ALLIANCE_COLOR);
        telemetryEx.update();


        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();
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

//            telemetryData.addData("Flywheel Velocity", robot.launchEncoder.getCorrectedVelocity());
//            telemetryData.addData("Flywheel Active Control", robot.launcher.getActiveControl());
//            telemetryData.addData("Flywheel Target Ball Velocity", robot.launcher.getTargetFlywheelVelocity());
//            telemetryData.addData("Flywheel Target", robot.launcher.getFlywheelTarget());
//            telemetryData.addData("Flywheel Ready", robot.launcher.flywheelReady());

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
    }

    public SequentialCommandGroup pathShoot(int pathStartingIndex, long timeout) {
        return new SequentialCommandGroup(
                new DriveTo(pathPoses.get(pathStartingIndex)).withTimeout(timeout).alongWith(
                        new InstantCommand(() -> robot.launcher.setLauncher(pathPoses.get(pathStartingIndex)))
                ),

                new ClearLaunch(true).raceWith(
                        new RunCommand(
                                () -> robot.drive.swerve.updateWithTargetVelocity(
                                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                                robot.drive.follower.calculate(robot.drive.getPose()),
                                                robot.drive.getPose().getRotation()
                                        )
                                )
                        )
                ),

                new InstantCommand(() -> robot.launcher.setRamp(false))
        );
    }

    public SequentialCommandGroup gateIntake(int pathStartingIndex, long timeout) {
        return new SequentialCommandGroup(
                new DriveTo(pathPoses.get(pathStartingIndex)).withTimeout(timeout),
                new SetIntake(Intake.MotorState.FORWARD),
                new WaitCommand(700)
        );
    }

    public SequentialCommandGroup pathIntake(int pathStartingIndex, long timeout) {
        return new SequentialCommandGroup(
                new SetIntake(Intake.MotorState.FORWARD),
                new DriveTo(pathPoses.get(pathStartingIndex), 0.5).withTimeout(timeout),
                new SetIntake(Intake.MotorState.STOP)
        );
    }
}