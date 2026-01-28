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
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commandbase.commands.DriveTo;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.commandbase.commands.StationaryAimbotFullLaunch;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.ArrayList;
import java.util.Arrays;

@Config
@Autonomous(name = "Gwi-Ma (quals close 18 Ball)", preselectTeleOp = "FullTeleOp", group = "Auto")
public class GwiMa extends CommandOpMode {
    public ElapsedTime timer;
    public static int REPEAT_TIMES = 2;
    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();
    public ArrayList<Pose2d> pathPoses;

    public void generatePath() {
        pathPoses = new ArrayList<>();
        pathPoses.add(new Pose2d(-43.24023076923076, 54.670769233076923, Math.toRadians(0))); // Starting Pose
        pathPoses.add(new Pose2d(-23.536231884057976, 12.869565217391301, Math.toRadians(0))); // Line 1
        pathPoses.add(new Pose2d(-26.550724637681157, -13.492753623188406, Math.toRadians(0))); // Line 2
        pathPoses.add(new Pose2d(-63.00420289855072, -13.492753623188406, Math.toRadians(0))); // Line 3
        pathPoses.add(new Pose2d(-33.55072463768116, -13.565217391304344, Math.toRadians(0))); // Line 4
        pathPoses.add(new Pose2d(-15.65217391304348, 9.623188405797105, Math.toRadians(0))); // Line 5
        pathPoses.add(new Pose2d(-35.13043478260869, -10.782608695652172, Math.toRadians(-35))); // Line 6
        pathPoses.add(new Pose2d(-57.31884057971015, -11.782608695652172, Math.toRadians(-35))); // Line 7
        pathPoses.add(new Pose2d(-35.13043478260869, -10.782608695652172, Math.toRadians(0))); // Line 8
        pathPoses.add(new Pose2d(-23.536231884057976, 12.637681159420286, Math.toRadians(0))); // Line 9
        pathPoses.add(new Pose2d(-56.23188405797101, 12.637681159420286, Math.toRadians(0))); // Line 10
        pathPoses.add(new Pose2d(-23.536231884057976, 12.637681159420286, Math.toRadians(0))); // Line 11
        pathPoses.add(new Pose2d(-58.18083216456287, -10.799812996727443, Math.toRadians(-30))); // Line 12
        pathPoses.add(new Pose2d(-23.806638616175785, 17.30341280972417, Math.toRadians(-30))); // Line 13
        pathPoses.add(new Pose2d(-34.02599345488546, 13.122767648433847, Math.toRadians(-45))); // Line 14

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
        robot.turret.setTurret(GOAL_LOCK_CONTROL, 0);

        // Schedule the full auto
        schedule(
                new SequentialCommandGroup(
                        // Set starting pose
                        new InstantCommand(),
                        new InstantCommand(() -> robot.drive.setPose(pathPoses.get(0))),

                        // preload
                        pathShoot(1, 2000),

                        // 2nd spike mark
                        new DriveTo(pathPoses.get(2)).withTimeout(1267),
                        pathIntake(3, 1200),

                        new DriveTo(pathPoses.get(4)).withTimeout(1267),
                        pathShoot(5, 1500),

                        // gate intake 1
                        new DriveTo(pathPoses.get(6)).withTimeout(1267),
                        gateIntake(7, 1267),

                        new DriveTo(pathPoses.get(8)).withTimeout(1267),
                        pathShoot(9, 1500),

                        // 1st spike mark
                        pathIntake(10, 1400),
                        pathShoot(11, 1600),

                        // gate cycles
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        gateIntake(12, 1467),
                                        pathShoot(13, 1600)
                                ),
                                REPEAT_TIMES
                        ),

                        // park
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
        }

        if (gamepad1.right_stick_button) {
            TURRET_SYNCED = false;
            robot.turret.resetTurretEncoder();
            robot.pinpoint.resetPosAndIMU();
        }

        telemetryData.addData("TURRET_SYNCED", TURRET_SYNCED);
        telemetryData.addData("Alliance Color", ALLIANCE_COLOR);
        telemetryData.addData("REPEAT_TIMES", REPEAT_TIMES);
        telemetryData.update();

        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();
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
//
            telemetryData.addData("Robot Pose", robot.drive.getPose());
            telemetryData.addData("Robot Target", robot.drive.follower.getTarget());
            telemetryData.addData("atTarget", robot.drive.follower.atTarget());
            telemetryData.addData("Heading", robot.drive.getPose().getHeading());
            telemetryData.addData("Heading Coefficients", Arrays.toString(((PIDFController)robot.drive.follower.headingController).getCoefficients()));
            telemetryData.addData("Target Chassis Velocity", robot.drive.swerve.getTargetVelocity());
//
            telemetryData.addData("Turret State", Turret.turretState);
            telemetryData.addData("Turret Target", robot.turret.getTarget());
            telemetryData.addData("Turret readyToLaunch", robot.turret.readyToLaunch());
            telemetryData.addData("Turret Position", robot.turret.getPosition());
            telemetryData.update();

//            telemetryData.addData("Flywheel Velocity", robot.launchEncoder.getCorrectedVelocity());
//            telemetryData.addData("Flywheel Active Control", robot.launcher.getActiveControl());
//            telemetryData.addData("Flywheel Target Ball Velocity", robot.launcher.getTargetFlywheelVelocity());
//            telemetryData.addData("Flywheel Target", robot.launcher.getFlywheelTarget());
//            telemetryData.addData("Flywheel Ready", robot.launcher.flywheelReady());

//            telemetryData.addData("Target Chassis Velocity", robot.drive.swerve.getTargetVelocity());

            robot.profiler.end("TelemetryData");
        }

        telemetryData.addData("Swerve Target Vel", robot.drive.swerve.getTargetVelocity());
        telemetryData.addData("Robot Pose", robot.drive.getPose());

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

                new StationaryAimbotFullLaunch().raceWith(
                        new RepeatCommand(
                                new InstantCommand(() -> robot.drive.swerve.updateWithXLock())
                        ),
                        new WaitCommand(3000)
                )
        );
    }

    public SequentialCommandGroup gateIntake(int pathStartingIndex, long timeout) {
        return new SequentialCommandGroup(
                new DriveTo(pathPoses.get(pathStartingIndex)).withTimeout(timeout),
                new SetIntake(Intake.MotorState.FORWARD),
                new ParallelRaceGroup(
                        new WaitCommand(1670)
//                                new WaitUntilCommand(() -> robot.intake.transferFull())
                )
        );
    }

    public SequentialCommandGroup pathIntake(int pathStartingIndex, long timeout) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SetIntake(Intake.MotorState.FORWARD),
                        new DriveTo(pathPoses.get(pathStartingIndex), 0.5).withTimeout(timeout)
                ),
                new SetIntake(Intake.MotorState.STOP)
        );
    }
}