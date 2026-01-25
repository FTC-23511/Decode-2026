package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.GOAL_LOCK_CONTROL;
import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.*;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commandbase.commands.*;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Robot;
import java.util.ArrayList;
import java.util.Arrays;

@Config
@Autonomous(name = "Gwi-Ma (close 18 Ball gate intake)", preselectTeleOp = "AAAFullTeleOp", group = "Auto")
public class GwiMa extends CommandOpMode {
    public ElapsedTime timer;
    public static boolean GATE_OPEN = false;
    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();
    public ArrayList<Pose2d> pathPoses;

    public void generatePath() {
        pathPoses = new ArrayList<>();
        pathPoses.add(new Pose2d(-47.76923076923076, 59.3076923076923, Math.toRadians(53))); // Starting Pose
        pathPoses.add(new Pose2d(-21.461538461538467, 13.615384615384613, Math.toRadians(0))); // Line 1
        pathPoses.add(new Pose2d(-29.538461538461537, -9.692307692307693, Math.toRadians(0))); // Line 2
        pathPoses.add(new Pose2d(-58.38461538461539, -14.999999999999996, Math.toRadians(0))); // Line 3
        pathPoses.add(new Pose2d(-15.461538461538463, 3.92307692307692, Math.toRadians(0))); // Line 4
        pathPoses.add(new Pose2d(-57.3076923076923, -4.15384615384616, Math.toRadians(0))); // Line 5
        pathPoses.add(new Pose2d(-54, -4.15384615384616, Math.toRadians(0))); // Line 6
        pathPoses.add(new Pose2d(-55.38461538461539, -14.538461538461533, Math.toRadians(-19))); // Line 7
        pathPoses.add(new Pose2d(-61.15384615384616, -14.538461538461533, Math.toRadians(-19))); // Line 8
        pathPoses.add(new Pose2d(-15.461538461538463, 3.92307692307692, Math.toRadians(0))); // Line 9
        pathPoses.add(new Pose2d(-27, 12.923076923076923, Math.toRadians(0))); // Line 10
        pathPoses.add(new Pose2d(-54.92307692307692, 12.461538461538467, Math.toRadians(0))); // Line 11
        pathPoses.add(new Pose2d(-21.461538461538467, 13.615384615384613, Math.toRadians(0))); // Line 12

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
        robot.turret.setTurret(GOAL_LOCK_CONTROL, (Math.PI / 2) * ALLIANCE_COLOR.getMultiplier());

        // Schedule the full auto
        schedule(
                new SequentialCommandGroup(
                        // Set starting pose
                        new InstantCommand(),
                        new InstantCommand(() -> robot.drive.setPose(pathPoses.get(0))),

                        // preload
                        pathShoot(1, 2000),

                        // 2nd spike mark
                        pathIntake(2, 1000),
                        pathShoot(4, 1200),

                        // gate intake 1
                        new DriveTo(pathPoses.get(5), 0.674).withTimeout(1000),
                        new DriveTo(pathPoses.get(6)).withTimeout(200),
                        new SetIntake(Intake.MotorState.FORWARD),
                        new DriveTo(pathPoses.get(7)).withTimeout(400),
                        new DriveTo(pathPoses.get(8)).withTimeout(800),
                        pathShoot(9, 2000),

                        // 1st spike mark
                        pathIntake(10, 1100),
                        pathShoot(11, 1470),

                        // park
                        new DriveTo(pathPoses.get(pathPoses.size() - 1))
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

        if (gamepad1.right_stick_button) {
            TURRET_SYNCED = false;
            robot.turret.resetTurretEncoder();
            robot.pinpoint.resetPosAndIMU();
        }

        telemetryData.addData("Gate Open", GATE_OPEN);
        telemetryData.addData("TURRET_SYNCED", TURRET_SYNCED);
        telemetryData.addData("Alliance Color", ALLIANCE_COLOR);
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

    public SequentialCommandGroup pathIntake(int pathStartingIndex, long timeout) {
        return new SequentialCommandGroup(
                new DriveTo(pathPoses.get(pathStartingIndex)).withTimeout(timeout),
                new ParallelCommandGroup(
                        new SetIntake(Intake.MotorState.FORWARD),
                        new DriveTo(pathPoses.get(pathStartingIndex + 1), 0.5).withTimeout(1400)
                ),
                new SetIntake(Intake.MotorState.STOP)
        );
    }

    public SequentialCommandGroup instantPathIntake(int pathStartingIndex, long timeout) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SetIntake(Intake.MotorState.FORWARD),
                        new DriveTo(pathPoses.get(pathStartingIndex), 0.5).withTimeout(timeout)
                ),
                new SetIntake(Intake.MotorState.STOP)
        );
    }
}