package org.firstinspires.ftc.teamcode.opmode;

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
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commandbase.commands.ClearLaunch;
import org.firstinspires.ftc.teamcode.commandbase.commands.DriveTo;
import org.firstinspires.ftc.teamcode.commandbase.commands.FullAim;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.ArrayList;
import java.util.Arrays;

@Config
@Autonomous(name = "Jinu3 (close 9 Ball double gate)", preselectTeleOp = "AAAFullTeleOp", group = "Auto")
public class Jinu3 extends CommandOpMode {
    public ElapsedTime timer;
    public static boolean GATE_OPEN = true;
    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();
    public ArrayList<Pose2d> pathPoses;

    public void generatePath() {
        pathPoses = new ArrayList<>();
        pathPoses.add(new Pose2d(-48.536741214057514, 58.42811501597444, Math.toRadians(53)));  // 0: Set Pose
        pathPoses.add(new Pose2d(-13.607898448419048, 12.490832157968967, Math.toRadians(15))); // 1: Shoot Preload
        pathPoses.add(new Pose2d(-57.04792332268371,  12.490832157968967, Math.toRadians(0)));  // 2: Spike 1 Intake
        pathPoses.add(new Pose2d(-49.150916784203105, 1.1170662905500706, Math.toRadians(0)));  // 3: Gate Open Move 1
        pathPoses.add(new Pose2d(-59.02779552715655,  0.5003897763589601, Math.toRadians(0)));  // 4: Gate Open Move 2
        pathPoses.add(new Pose2d(-13.607898448519048, 12.490832157968967, Math.toRadians(15))); // 5: Shoot 2
        pathPoses.add(new Pose2d(-24.16925246826516, -13.700832157968967, Math.toRadians(0)));  // 6: Spike 2 Intake
        pathPoses.add(new Pose2d(-60.9308885754584, -13.700832157968967, Math.toRadians(0)));   // 7: Spike 2 Intake
        pathPoses.add(new Pose2d(-34.1212976022567, -13.700832157968967, Math.toRadians(0)));   // 8: Transition
        pathPoses.add(new Pose2d(-59.02779552715655, -0.9201277955271507, Math.toRadians(0)));   // 9: Gate again
        pathPoses.add(new Pose2d(-13.607898448519048, 12.490832157968967, Math.toRadians(15))); // 10: Shoot 3
        pathPoses.add(new Pose2d(-25.184767277856132, -36.86318758815233, Math.toRadians(0)));  // 11: Spike 3 Intake
        pathPoses.add(new Pose2d(-63.774330042313125, -36.86318758815233, Math.toRadians(0)));  // 12: Spike 3 Intake
        pathPoses.add(new Pose2d(-13.607898448519048, 12.490832157968967, Math.toRadians(15))); // 13: Shoot 4
        pathPoses.add(new Pose2d(-28.96614950634697, -2.7108603667136748, Math.toRadians(0)));   // 14: Park

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
        robot.turret.setTurret(GOAL_LOCK_CONTROL, 0);

        // Schedule the full auto
        schedule(
                new SequentialCommandGroup(
                        // Set starting pose
                        new InstantCommand(),
                        new InstantCommand(() -> robot.drive.setPose(pathPoses.get(0))),

                        // Score Preload
                        new DriveTo(pathPoses.get(1)).alongWith(
                                new InstantCommand(() -> robot.launcher.setLauncher(pathPoses.get(1)))
                        ).withTimeout(2000),
                        new FullAim().withTimeout(1000),
                        new WaitCommand(250),
                        new ClearLaunch(true).alongWith(
                                new InstantCommand(
                                        () -> robot.drive.swerve.updateWithXLock() // Lock the drivetrain wheel to an X shape reduce how much we can be pushed)
                                )
                        ),
                        new WaitCommand(300),

                        // Spike 1 Sequence
                        new ParallelCommandGroup(
                                new SetIntake(Intake.MotorState.FORWARD),
                                new DriveTo(pathPoses.get(2), 0.5).withTimeout(1670)
                        ),
                        new SetIntake(Intake.MotorState.STOP),

                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new DriveTo(pathPoses.get(3)).withTimeout(767),
                                        new DriveTo(pathPoses.get(4)).withTimeout(876)
                                ),
                                new InstantCommand(),
                                () -> GATE_OPEN
                        ),

                        new WaitCommand(300),

                        pathShoot(5, 1500),

                        // Spike 2 Sequence
                        pathIntake(6, 1267),
                        new DriveTo(pathPoses.get(8)).withTimeout(800),

                        new ConditionalCommand(
                                new DriveTo(pathPoses.get(9)).withTimeout(876),
                                new InstantCommand(),
                                () -> GATE_OPEN
                        ),

                        new WaitCommand(500),

                        pathShoot(10, 1500),

                        // Park
                        new ClearLaunch(true),
                        new DriveTo(pathPoses.get(14)).alongWith(
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
                        new DriveTo(pathPoses.get(pathStartingIndex + 1), 0.5).withTimeout(1367)
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