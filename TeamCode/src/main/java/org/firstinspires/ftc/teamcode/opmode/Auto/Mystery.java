package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.ANGLE_CONTROL;
import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.swerve.coaxial.CoaxialSwerveModule;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commandbase.commands.ClearLaunch;
import org.firstinspires.ftc.teamcode.commandbase.commands.DriveTo;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.ArrayList;

@Config
@Autonomous(name = "Mystery (6 ball)")
public class Mystery extends CommandOpMode {
    public ElapsedTime timer;

    TelemetryData telemetryData = new TelemetryData(
            new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())
    );

    private final Robot robot = Robot.getInstance();

    public ArrayList<Pose2d> pathPoses;
    public void generatePath() {
        pathPoses = new ArrayList<>();

        pathPoses = new ArrayList<>();
        pathPoses.add(new Pose2d(-47.407408311631944, 58.3111111111111, Math.toRadians(144))); // Starting Pose
        pathPoses.add(new Pose2d(-38.33914421553091, 52.37400950871633, Math.toRadians(159))); // Line 1
        pathPoses.add(new Pose2d(-20.210776545166404, 11.5, Math.toRadians(0))); // Line 2
        pathPoses.add(new Pose2d(-50.595879556259906, 11.5, Math.toRadians(0))); // Line 3
        pathPoses.add(new Pose2d(-38.33914421553091, 52.37400950871633, Math.toRadians(159))); // Line 4

        if (ALLIANCE_COLOR.equals(AllianceColor.RED)) {
            for (Pose2d pose : pathPoses) {
                pose.mirror();
            }
        }
    }

    @Override
    public void initialize() {
        generatePath();
        timer = new ElapsedTime();

        // Must have for all opModes
        OP_MODE_TYPE = OpModeType.AUTO;

        // Resets the command scheduler
        super.reset();

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);

        robot.launcher.setHood(MIN_HOOD_SERVO_POS);
        robot.launcher.setRamp(true);
        robot.intake.setPivot(Intake.PivotState.TRANSFER);
        robot.turret.setTurret(ANGLE_CONTROL, 0);

        // Schedule the full auto
        // TODO: FIGURE OUT WHY WE NEED A BURNER INSTANT COMMAND
        schedule(
                new SequentialCommandGroup(
                        new InstantCommand(),
                        new InstantCommand(() -> robot.turret.setTurret(Turret.TurretState.OFF, 0)),
                        new InstantCommand(() -> robot.drive.setPose(pathPoses.get(0))),

                        pathShoot(pathPoses.get(1)),

                        new DriveTo(pathPoses.get(2)).withTimeout(5000),
                        new SetIntake(Intake.MotorState.FORWARD, Intake.PivotState.FORWARD),

                        new DriveTo(pathPoses.get(3)).withTimeout(3000),

                        pathShoot(pathPoses.get(4)),
                        new ClearLaunch(true)
                )
        );
    }

    @Override
    public void run() {
        // DO NOT REMOVE
        super.run();
        telemetryData.update();

        // Update any constants that are being updated by FTCDash - used for tuning
        for (CoaxialSwerveModule module : robot.drive.swerve.getModules()) {
            module.setSwervoPIDF(SWERVO_PIDF_COEFFICIENTS);
        }
        ((PIDFController) robot.drive.follower.xController).setCoefficients(XY_COEFFICIENTS);
        ((PIDFController) robot.drive.follower.yController).setCoefficients(XY_COEFFICIENTS);
        ((PIDFController) robot.drive.follower.headingController).setCoefficients(HEADING_COEFFICIENTS);

        telemetryData.addData("Loop Time", timer.milliseconds());
        timer.reset();

        telemetryData.addData("Heading", robot.drive.getPose().getHeading());
        telemetryData.addData("Robot Pose", robot.drive.getPose());
        telemetryData.addData("Target Pose", robot.drive.follower.getTarget());
        telemetryData.addData("Calculated", robot.drive.follower.calculate(robot.drive.getPose()));

        telemetryData.addData("X Error", robot.drive.follower.getError().getTranslation().getX());
        telemetryData.addData("Y Error", robot.drive.follower.getError().getTranslation().getY());
        telemetryData.addData("Heading Error", robot.drive.follower.getError().getRotation().getAngle(AngleUnit.RADIANS));

        telemetryData.addData("Target Chassis Velocity", robot.drive.swerve.getTargetVelocity());
        telemetryData.addData("FR Module", robot.drive.swerve.getModules()[0].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[0].getPowerTelemetry());
        telemetryData.addData("FL Module", robot.drive.swerve.getModules()[1].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[1].getPowerTelemetry());
        telemetryData.addData("BL Module", robot.drive.swerve.getModules()[2].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[2].getPowerTelemetry());
        telemetryData.addData("BR Module", robot.drive.swerve.getModules()[3].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[3].getPowerTelemetry());

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        telemetryData.update();
    }

    @Override
    public void end() {
        END_POSE = robot.drive.getPose();
    }

    public Command pathShoot(Pose2d pose2d) {
        return new SequentialCommandGroup(
                new DriveTo(pose2d).withTimeout(5000).alongWith(new InstantCommand(() -> robot.launcher.setFlywheel(LAUNCHER_CLOSE_VELOCITY, true))),
                new InstantCommand(() -> robot.turret.setTurret(ANGLE_CONTROL, 0)),
                new InstantCommand(() -> robot.readyToLaunch = true),
                new ClearLaunch(true)
        );
    }
}