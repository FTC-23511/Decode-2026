package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.swerve.coaxial.CoaxialSwerveModule;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commandbase.commands.ClearLaunch;
import org.firstinspires.ftc.teamcode.commandbase.commands.DriveTo;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.ArrayList;

@Config
@Deprecated
@Disabled
@Autonomous(name = "LM0Auto")
public class Baby extends CommandOpMode {
    public ElapsedTime timer;

    TelemetryData telemetryData = new TelemetryData(
            new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())
    );

    private final Robot robot = Robot.getInstance();

    public ArrayList<Pose2d> pathPoses;
    public void generatePath() {
        pathPoses = new ArrayList<>();

        pathPoses.add(new Pose2d(0, 0, Math.toRadians(0))); // Starting Pose
        pathPoses.add(new Pose2d(-20, 0, Math.toRadians(0))); // Line 1
//        pathPoses.add(new Pose2d(27.84152139461173, -69.10618066561012, Math.toRadians(-54.046))); // Line 2

//        pathPoses.add(new Pose2d(25.90174326465927, 129.16640253565768, Math.toRadians(144.046))); // Starting Pose
//        pathPoses.add(new Pose2d(-27.8415213946, 116.38668779714739, Math.toRadians(157))); // Line 1
//        pathPoses.add(new Pose2d(53.743264659271, 60.06022187004756, Math.toRadians(90))); // Line 2
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

        // Schedule the full auto
        robot.drive.setPose(pathPoses.get(0));

        schedule(
                new SequentialCommandGroup(
                        new WaitCommand(2000),
                        new InstantCommand(() -> robot.launcher.setHood(MIN_HOOD_ANGLE)),
                        new InstantCommand(() -> robot.launcher.setFlywheel(LAUNCHER_CLOSE_VELOCITY, true)),
                        new WaitCommand(3000),
                        new InstantCommand(() -> robot.launcher.setRamp(true)),
                        new WaitCommand(500),
                        new ClearLaunch(),
                        new WaitCommand(500),
                        new SetIntake(Intake.MotorState.REVERSE, Intake.PivotState.HOLD),
                        new WaitCommand(200),
                        new ClearLaunch()
//                        new DriveTo(pathPoses.get(1))
                ).alongWith(
                        new RunCommand(
                                () -> schedule(new DriveTo(pathPoses.get(1)))
                        )
                )
        );
    }

    @Override
    public void run() {
        // DO NOT REMOVE
        super.run();

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

}