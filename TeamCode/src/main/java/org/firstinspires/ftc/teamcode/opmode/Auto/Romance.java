package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.ANGLE_CONTROL;
import static org.firstinspires.ftc.teamcode.globals.Constants.ALLIANCE_COLOR;
import static org.firstinspires.ftc.teamcode.globals.Constants.AllianceColor;
import static org.firstinspires.ftc.teamcode.globals.Constants.END_POSE;
import static org.firstinspires.ftc.teamcode.globals.Constants.HEADING_COEFFICIENTS;
import static org.firstinspires.ftc.teamcode.globals.Constants.LAUNCHER_CLOSE_VELOCITY;
import static org.firstinspires.ftc.teamcode.globals.Constants.LAUNCHER_FAR_VELOCITY;
import static org.firstinspires.ftc.teamcode.globals.Constants.MAX_HOOD_ANGLE;
import static org.firstinspires.ftc.teamcode.globals.Constants.MIN_HOOD_SERVO_POS;
import static org.firstinspires.ftc.teamcode.globals.Constants.OP_MODE_TYPE;
import static org.firstinspires.ftc.teamcode.globals.Constants.OpModeType;
import static org.firstinspires.ftc.teamcode.globals.Constants.PROBLEMATIC_TELEMETRY;
import static org.firstinspires.ftc.teamcode.globals.Constants.SWERVO_PIDF_COEFFICIENTS;
import static org.firstinspires.ftc.teamcode.globals.Constants.XY_COEFFICIENTS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.swerve.coaxial.CoaxialSwerveModule;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commandbase.commands.ClearLaunch;
import org.firstinspires.ftc.teamcode.commandbase.commands.DriveTo;
import org.firstinspires.ftc.teamcode.commandbase.commands.PrepDriveTo;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.ArrayList;

@Config
@Autonomous(name = "Romance (close far auto)", preselectTeleOp = "AAAFullTeleOp")
public class Romance extends CommandOpMode {
    public ElapsedTime timer;

    TelemetryData telemetryData = new TelemetryData(
            new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())
    );

    private final Robot robot = Robot.getInstance();

    public ArrayList<Pose2d> pathPoses;
    public void generatePath() {
        pathPoses = new ArrayList<>();

        pathPoses.add(new Pose2d(-47.40, 58.31, Math.toRadians(144))); // Starting Pose
        pathPoses.add(new Pose2d(-15.0, 16.0, Math.toRadians(50))); // Line 1
        pathPoses.add(new Pose2d(-16.0, 24.0, Math.toRadians(60))); // Line 2
        pathPoses.add(new Pose2d(-16.7, 11.0, Math.toRadians(10))); // Line 3
        pathPoses.add(new Pose2d(-51.0, 11.0, Math.toRadians(10))); // Line 4
        pathPoses.add(new Pose2d(-15.0, 16.0, Math.toRadians(50))); // Line 5
        pathPoses.add(new Pose2d(-12.7, -12.0, Math.toRadians(10))); // Line 6
        pathPoses.add(new Pose2d(-55.0, -12.0, Math.toRadians(10))); // Line 7
        pathPoses.add(new Pose2d(-32.0, -12.0, Math.toRadians(10))); // Line 8
        pathPoses.add(new Pose2d(-15.0, 16.0, Math.toRadians(50))); // Line 9
        pathPoses.add(new Pose2d(-30.0, 52.37, Math.toRadians(0))); // Line 10

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

        robot.launcher.setHood(MAX_HOOD_ANGLE);
        robot.launcher.setRamp(true);
        robot.intake.setPivot(Intake.PivotState.HOLD);
        robot.turret.setTurret(ANGLE_CONTROL, 0);

        // Schedule the full auto
        // TODO: FIGURE OUT WHY WE NEED A BURNER INSTANT COMMAND
        schedule(
                new SequentialCommandGroup(
                        // init
                        new InstantCommand(),
                        new InstantCommand(() -> robot.turret.setTurret(ANGLE_CONTROL, ((Math.PI/2 + 0.1) * ALLIANCE_COLOR.getMultiplier()))),
                        new InstantCommand(() -> robot.drive.setPose(pathPoses.get(0))),

                        // preload
                        pathShoot(1, 2500),
                        new WaitCommand(1000),

                        // spike 1
                        new DriveTo(pathPoses.get(2)).withTimeout(300),
                        pathIntake(3, 1867),
                        pathShoot(5, 2250),

                        // spike 2
                        pathIntake(6, 2267),
//                        new DriveTo(pathPoses.get(8)).withTimeout(670),
                        pathShoot(9, 3000),
                        new ClearLaunch(true),
                        
                        new DriveTo(pathPoses.get(10)) // park
                )
        );
    }

    @Override
    public void initialize_loop() {
        telemetryData.addData("ALLIANCE_COLOR", ALLIANCE_COLOR);
        telemetryData.update();
    }

    @Override
    public void run() {
        // DO NOT REMOVE
        robot.updateLoop(telemetryData);

        // Update any constants that are being updated by FTCDash - used for tuning
        for (CoaxialSwerveModule module : robot.drive.swerve.getModules()) {
            module.setSwervoPIDF(SWERVO_PIDF_COEFFICIENTS);
        }
        ((PIDFController) robot.drive.follower.xController).setCoefficients(XY_COEFFICIENTS);
        ((PIDFController) robot.drive.follower.yController).setCoefficients(XY_COEFFICIENTS);
        ((PIDFController) robot.drive.follower.headingController).setCoefficients(HEADING_COEFFICIENTS);

        telemetryData.addData("Loop Time", timer.milliseconds());
        timer.reset();

        if (PROBLEMATIC_TELEMETRY) {
            robot.profiler.start("High TelemetryData");

            telemetryData.addData("Heading", robot.drive.getPose().getHeading());
            telemetryData.addData("Robot Pose", robot.drive.getPose());
            telemetryData.addData("Turret Position", robot.turret.getPosition());
            telemetryData.addData("Flywheel Velocity", robot.launchEncoder.getCorrectedVelocity());
            telemetryData.addData("Intake overCurrent", ((MotorEx) robot.intakeMotors.getMotor()).isOverCurrent());
            telemetryData.addData("FR Module", robot.drive.swerve.getModules()[0].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[0].getPowerTelemetry());
            telemetryData.addData("FL Module", robot.drive.swerve.getModules()[1].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[1].getPowerTelemetry());
            telemetryData.addData("BL Module", robot.drive.swerve.getModules()[2].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[2].getPowerTelemetry());
            telemetryData.addData("BR Module", robot.drive.swerve.getModules()[3].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[3].getPowerTelemetry());

            robot.profiler.end("High TelemetryData");
        }

        robot.profiler.start("Low TelemetryData");
        telemetryData.addData("Robot Target", robot.drive.follower.getTarget());
        telemetryData.addData("atTarget", robot.drive.follower.atTarget());
        telemetryData.addData("X Error", robot.drive.follower.getError().getTranslation().getX());
        telemetryData.addData("Y Error", robot.drive.follower.getError().getTranslation().getY());
        telemetryData.addData("Heading Error", robot.drive.follower.getError().getRotation().getAngle(AngleUnit.RADIANS));

        telemetryData.addData("Turret State", Turret.turretState);
        telemetryData.addData("Turret Target", robot.turret.getTarget());
        telemetryData.addData("Turret readyToLaunch", robot.turret.readyToLaunch());

        telemetryData.addData("Flywheel Active Control", robot.launcher.getActiveControl());
        telemetryData.addData("Flywheel Target Ball Velocity", robot.launcher.getTargetFlywheelVelocity());
        telemetryData.addData("Flywheel Target", robot.launcher.getFlywheelTarget());

        telemetryData.addData("Intake Motor State", Intake.motorState);
        telemetryData.addData("Intake Jammed", robot.intake.intakeJammed);

        telemetryData.addData("Target Chassis Velocity", robot.drive.swerve.getTargetVelocity());

        telemetryData.addData("Sigma", "Polar");

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        telemetryData.update();
    }

    @Override
    public void end() {
        END_POSE = robot.drive.getPose();
    }

    public SequentialCommandGroup pathShoot(int pathStartingIndex, long timeout) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new DriveTo(pathPoses.get(pathStartingIndex)).withTimeout(timeout),
                        new InstantCommand(() -> robot.launcher.setFlywheel(5.67, true))
                ),
                new InstantCommand(() -> robot.turret.setTurret(ANGLE_CONTROL, ((Math.PI/2 - 0.1) * ALLIANCE_COLOR.getMultiplier()))),
                new InstantCommand(() -> robot.readyToLaunch = true),
                new ClearLaunch(true).alongWith(
                        new PrepDriveTo(pathPoses.get(pathStartingIndex + 1))
                )
        );
    }

    public SequentialCommandGroup pathIntake(int pathStartingIndex, long timeout) {
        return new SequentialCommandGroup(
                new DriveTo(pathPoses.get(pathStartingIndex)).withTimeout(timeout),
                new SetIntake(Intake.MotorState.FORWARD, Intake.PivotState.FORWARD),

                new DriveTo(pathPoses.get(pathStartingIndex+1)).withTimeout(2467)
        );
    }
}