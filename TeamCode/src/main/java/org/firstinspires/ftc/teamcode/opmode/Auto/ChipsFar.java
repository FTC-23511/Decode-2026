package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.ANGLE_CONTROL;
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
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.TelemetryEx;

import org.firstinspires.ftc.teamcode.commandbase.commands.ContinuousClearLaunch;
import org.firstinspires.ftc.teamcode.commandbase.commands.DriveTo;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.ArrayList;
import java.util.Arrays;

@Config
@Autonomous(name = "ChipsFar", preselectTeleOp = "FullTeleOp", group = "Auto")
public class ChipsFar extends CommandOpMode {
    public ElapsedTime timer;

    public ElapsedTime autoTimer;
    public static int REPEAT_TIMES = 5;
    public static boolean GATE_OPEN = false;
    private boolean PATH_ALTERNATE = true;
    TelemetryEx telemetryEx = new TelemetryEx(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();
    public ArrayList<Pose2d> pathPoses;

    public void generatePath() {
        pathPoses = new ArrayList<>();
        pathPoses.add(new Pose2d(-15.25, -64, Math.toRadians(0))); // Starting Pose
        pathPoses.add(new Pose2d(-26.420168067226896, -35.89915966386555, Math.toRadians(0))); // Line 1
        pathPoses.add(new Pose2d(-62.92436974789916, -35.89915966386555, Math.toRadians(0))); // Line 2
        pathPoses.add(new Pose2d(-22.903937007874017, -62.02204724409449, Math.toRadians(0))); // Line 3
        pathPoses.add(new Pose2d(-62.36220472440945, -62.475590551181114, Math.toRadians(0))); // Line 4
        pathPoses.add(new Pose2d(-55.55905511811024, -63.15590551181103, Math.toRadians(0))); // Line 5
        pathPoses.add(new Pose2d(-63.7228346456693, -59.30078740157481, Math.toRadians(0))); // Line 6
        pathPoses.add(new Pose2d(-21.08976377952756, -59.52755905511811, Math.toRadians(0))); // Line 7
        pathPoses.add(new Pose2d(-60.32125984251968, -54.31181102362204, Math.toRadians(20))); // Line 8
        pathPoses.add(new Pose2d(-62.135433070866135, -71.09291338582676, Math.toRadians(0))); // Line 9
        pathPoses.add(new Pose2d(-22.903937007874017, -62.02204724409449, Math.toRadians(0))); // Line 10
        pathPoses.add(new Pose2d(-60.548031496063, -46.82834645669291, Math.toRadians(340))); // Line 11
        pathPoses.add(new Pose2d(-60.32125984251968, -28.23307086614173, Math.toRadians(340))); // Line 12
        pathPoses.add(new Pose2d(-21.08976377952756, -59.30078740157481, Math.toRadians(340))); // Line 13
        pathPoses.add(new Pose2d(-29.848739495798316, -52.23529411764707, Math.toRadians(315))); // Line 14
    

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
        robot.turret.setTurret(ANGLE_CONTROL, Math.toRadians(120) * ALLIANCE_COLOR.getMultiplier());
        robot.turret.setTurret(GOAL_LOCK_CONTROL, 0);

        // Schedule the full auto
        schedule(
                new SequentialCommandGroup(
                        // Set starting pose
                        new InstantCommand(),
                        new InstantCommand(() -> robot.drive.setPose(pathPoses.get(0))),
                        new InstantCommand(() -> robot.drive.swerve.setMaxSpeed(0.9)),

                        // preload
                        pathShoot(0, 1600),

                        // 3rd spike mark
                        new DriveTo(pathPoses.get(1)).withTimeout(967),
                        pathIntake(2, 1670),
                        pathShoot(3, 1600),

                        // hp line intake
                        new SequentialCommandGroup(
                                new SetIntake(Intake.MotorState.FORWARD),
                                new DriveTo(pathPoses.get(4), 0.6).withTimeout(1670),
                                new DriveTo(pathPoses.get(5), 0.5).withTimeout(500),
                                new DriveTo(pathPoses.get(6), 0.5).withTimeout(1000)
                        ).interruptOn(() -> robot.intake.transferFull()),

                        new SetIntake(Intake.MotorState.STOP),
                        pathShoot(7, 1600),

                        // hp intake™
                        new SetIntake(Intake.MotorState.FORWARD),
                        new DriveTo(pathPoses.get(8), 0.6).withTimeout(1670),
                        new DriveTo(pathPoses.get(9), 0.8).withTimeout(567),
                        pathShoot(10, 1600),

                        // intake cycles
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        new SetIntake(Intake.MotorState.FORWARD),
                                        new DriveTo(pathPoses.get(11), 0.6).withTimeout(1670),
                                        new DriveTo(pathPoses.get(12), 0.8).withTimeout(1670),
                                        new SetIntake(Intake.MotorState.STOP),

                                        pathShoot(13, 1550)
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

        if (gamepad1.right_stick_button) {
//            robot.pinpoint.resetPosAndIMU();
            TURRET_SYNCED = false;
            robot.turret.resetTurretEncoder();
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
                new ParallelCommandGroup(
                        new DriveTo(pathPoses.get(pathStartingIndex)).withTimeout(timeout),
                        new WaitCommand(timeout)
                ).deadlineWith(new ContinuousClearLaunch()),

                new InstantCommand(() -> robot.launcher.setRamp(false))
        );
    }

    public SequentialCommandGroup gateIntake(int pathStartingIndex, long timeout) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new DriveTo(pathPoses.get(pathStartingIndex)).withTimeout(timeout),
                        new InstantCommand(() -> robot.drive.swerve.setMaxSpeed(0.4)).beforeStarting(new WaitCommand(900))
                ),
                new SetIntake(Intake.MotorState.FORWARD),
                new WaitCommand(600),
                new InstantCommand(() -> robot.drive.swerve.setMaxSpeed(0.67))
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