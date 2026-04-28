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
@Autonomous(name = "TwentyOneFar", preselectTeleOp = "FullTeleOp", group = "Auto")
public class TwentyOneFar extends CommandOpMode {
    public ElapsedTime timer;

    public ElapsedTime autoTimer;
    public static int REPEAT_TIMES = 3;
    public static boolean INTAKE_THIRD_SPIKE = true;
    TelemetryEx telemetryEx = new TelemetryEx(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();
    public ArrayList<Pose2d> pathPoses;

    public void generatePath() {
        pathPoses = new ArrayList<>();
        pathPoses.add(new Pose2d(-18.5, -64, Math.toRadians(0))); // Starting Pose
        pathPoses.add(new Pose2d(-24.610168067226898, -37.089159663865544, Math.toRadians(0))); // Line 1
        pathPoses.add(new Pose2d(-61.11436974789911, -37.089159663865544, Math.toRadians(0))); // Line 2
        pathPoses.add(new Pose2d(-21.09393700787402, -61.212047244094485, Math.toRadians(0))); // Line 3
        pathPoses.add(new Pose2d(-60.55220472440944, -61.66559055118111, Math.toRadians(0))); // Line 4
        pathPoses.add(new Pose2d(-53.749055118110824, -62.345905511811026, Math.toRadians(0))); // Line 5
        pathPoses.add(new Pose2d(-61.91283464566993, -58.490787401574806, Math.toRadians(0))); // Line 6
        pathPoses.add(new Pose2d(-19.279763779527567, -58.71755905511811, Math.toRadians(0))); // Line 7
        pathPoses.add(new Pose2d(-58.056872037914694, -50.161137440758296, Math.toRadians(323))); // Line 8
        pathPoses.add(new Pose2d(-58.51125984251968, -27.02307086614173, Math.toRadians(323))); // Line 9
        pathPoses.add(new Pose2d(-18.145905511811026, -59.62464566929134, Math.toRadians(345))); // Line 10
        pathPoses.add(new Pose2d(-28.038739495798318, -51.42529411764707, Math.toRadians(323))); // Line 11

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
        robot.readyToLaunch = true;

        Launcher.DISTANCE_OFFSET = -0.175;

        // Schedule the full auto
        schedule(
                new SequentialCommandGroup(
                        // Set starting pose
                        new InstantCommand(),
                        new InstantCommand(() -> robot.drive.setPose(pathPoses.get(0))),
                        new InstantCommand(() -> robot.drive.swerve.setMaxSpeed(0.75)),
                        new InstantCommand(() -> robot.readyToLaunch = true),

                        // preload
                        new ClearLaunch(true).beforeStarting(new WaitCommand(500)),

                        // 3rd spike mark
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new DriveTo(pathPoses.get(1)).withTimeout(767),
                                        new SetIntake(Intake.MotorState.FORWARD),
                                        new DriveTo(pathPoses.get(2), 0.4, 0.7)
                                                .withTimeout(1670)
                                                .interruptOn(() -> robot.intake.transferFull()),

                                        pathShoot(3, 1600)
                                ),
                                new InstantCommand(),
                                () -> INTAKE_THIRD_SPIKE
                        ),

                        // HP Ball
                        new SequentialCommandGroup(
                                new SetIntake(Intake.MotorState.FORWARD),
                                new DriveTo(pathPoses.get(4), 0.6, 1.0).withTimeout(1000),
                                new DriveTo(pathPoses.get(5), 0.6, 1.0).withTimeout(500),
                                new DriveTo(pathPoses.get(6), 0.6, 1.0).withTimeout(500)
                        ).interruptOn(() -> robot.intake.transferFull()),

                        pathShoot(7, 1600),

                        // intake cycles
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        new SequentialCommandGroup(
                                                new SetIntake(Intake.MotorState.FORWARD),
                                                new DriveTo(pathPoses.get(8), 0.5, 0.8).withTimeout(1200),
                                                new DriveTo(pathPoses.get(9), 0.3, 0.6).withTimeout(1567)
                                        ).interruptOn(() -> robot.intake.transferFull()),

                                        pathShoot(10, 1550)
                                ),
                                REPEAT_TIMES
                        ),

                        new SequentialCommandGroup(
                                new SetIntake(Intake.MotorState.FORWARD),
                                new DriveTo(pathPoses.get(4), 1.0).withTimeout(1167)
                        ).interruptOn(() -> robot.intake.transferFull()),

                        pathShoot(7, 1400),

                        // park + end
                        new DriveTo(pathPoses.get(pathPoses.size() - 1)).alongWith(
                                new InstantCommand(() -> robot.turret.setTurretPos(0.5, true)),
                                new InstantCommand(() -> robot.intake.setIntake(Intake.MotorState.STOP))
                        )
                )
        );
    }

    @Override
    public void initialize_loop() {
        if (gamepad1.dpadUpWasPressed()) {
            REPEAT_TIMES++;
        } else if (gamepad1.dpadDownWasPressed()) {
            REPEAT_TIMES--;
            REPEAT_TIMES = Math.max(1, REPEAT_TIMES);
        }

        if (gamepad1.dpadLeftWasPressed()) {
            Launcher.DISTANCE_OFFSET -= 0.005;
        } else if (gamepad1.dpadDownWasPressed()) {
            Launcher.DISTANCE_OFFSET += 0.005;
        }

        if (gamepad1.cross || gamepad1.triangle) {
            INTAKE_THIRD_SPIKE = true;
        } else if (gamepad1.circle || gamepad1.square) {
            INTAKE_THIRD_SPIKE = false;
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
        telemetryEx.addData("INTAKE_THIRD_SPIKE", INTAKE_THIRD_SPIKE);
        telemetryEx.addData("Launcher DISTANCE_OFFSET", Launcher.DISTANCE_OFFSET);
        robot.initializeLoop(gamepad1, telemetryEx);
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
            autoTimer = new ElapsedTime();
        }

        // Always log Loop Time


        if (PROBLEMATIC_TELEMETRY) {
            robot.profiler.start("TelemetryData");

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
        robot.updateLoop(null);
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
        return pathShoot(pathStartingIndex, timeout, true);
    }

    public SequentialCommandGroup pathShoot(int pathStartingIndex, long timeout, boolean stopInZone) {
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        new DriveTo(pathPoses.get(pathStartingIndex), 1.0).alongWith(
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
                        new ClearLaunch(true).beforeStarting(new WaitCommand(250))
                )
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