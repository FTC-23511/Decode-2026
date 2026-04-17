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
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.util.TelemetryEx;

import org.firstinspires.ftc.teamcode.commandbase.commands.ClearLaunch;
import org.firstinspires.ftc.teamcode.commandbase.commands.ContinuousClearLaunch;
import org.firstinspires.ftc.teamcode.commandbase.commands.DriveTo;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
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

    public static boolean GATE_OPEN = false;
    TelemetryEx telemetryEx = new TelemetryEx(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();
    public ArrayList<Pose2d> pathPoses;

    public void generatePath() {
        pathPoses = new ArrayList<>();

        pathPoses.add(new Pose2d(-45.3781512605042, 55.05882352941177, Math.toRadians(0))); // Starting Pose
        pathPoses.add(new Pose2d(-23.19327731092437, 12.100840336134446, Math.toRadians(0))); // Line 1
        pathPoses.add(new Pose2d(-56.0672268907563, 12.302521008403353, Math.toRadians(0))); // Line 2
        pathPoses.add(new Pose2d(-46.58823529411764, 0.20168067226890685, Math.toRadians(-0.001))); // Line 3
        pathPoses.add(new Pose2d(-56.26890756302521, 0.202, Math.toRadians(0))); // Line 4
        pathPoses.add(new Pose2d(-18.352941176470583, 8.26890756302521, Math.toRadians(0))); // Line 5
        pathPoses.add(new Pose2d(-30.655462184873954, -11.89915966386555, Math.toRadians(0))); // Line 6
        pathPoses.add(new Pose2d(-61.3109243697479, -12.100840336134457, Math.toRadians(0))); // Line 7
        pathPoses.add(new Pose2d(-37.310924369747895, -9.075630252100837, Math.toRadians(0))); // Line 8
        pathPoses.add(new Pose2d(-55.66386554621849, -2.4201680672268964, Math.toRadians(0))); // Line 9
        pathPoses.add(new Pose2d(-15.327731092436977, 6.4537815126050475, Math.toRadians(0))); // Line 10
        pathPoses.add(new Pose2d(-55.23188405797101, -4.7681159420289845, Math.toRadians(-10))); // Line 11
        pathPoses.add(new Pose2d(-60.30252100840335, -20.369747899159673, Math.toRadians(-45))); // Line 12
        pathPoses.add(new Pose2d(-25.210084033613448, 12.50420168067227, Math.toRadians(0))); // Line 13
        pathPoses.add(new Pose2d(-19.159663865546214, -2.0168067226890685, Math.toRadians(0))); // Line 14


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
                        pathSOTM(1, 1600),

                        // intake 1st spike
                        pathIntake(2, 967),

                        // open gate
                        new DriveTo(pathPoses.get(3)).withTimeout(1670),
                        new DriveTo(pathPoses.get(4)).withTimeout(600),

                        // shoot 1st spike
                        pathShoot(5, 1250),

                        // intake 2nd spike
                        new SetIntake(Intake.MotorState.FORWARD),

                        new DriveTo(pathPoses.get(6), 1.0).withTimeout(500),
                        pathIntake(7, 967),


                        // open gate
                        new DriveTo(pathPoses.get(8), 1.0).withTimeout(467),
                        new DriveTo(pathPoses.get(9)).withTimeout(1267),

                        // shoot 2nd spike
                        pathShoot(10, 900),
                        // gate intake cycles
                        new RepeatCommand(
                                new SequentialCommandGroup(

                                        new DriveTo(pathPoses.get(11)).withTimeout(1267),

                                        // Intake turns on, drives to 11, turns off upon arrival
                                        gateIntake(12, 1500),

                                        // Drives to 12 and shoots
                                        pathShoot(13, 1500)
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

    public SequentialCommandGroup pathSOTM(int pathStartingIndex, long timeout) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> robot.readyToLaunch = true),
                new ParallelCommandGroup(
                        new DriveTo(pathPoses.get(pathStartingIndex)),
                        new InstantCommand(() -> robot.launcher.setLauncher(pathPoses.get(pathStartingIndex))),
                        new ContinuousClearLaunch()
                ).withTimeout(timeout),

                new InstantCommand(() -> robot.launcher.setRamp(false))
        );
    }
}