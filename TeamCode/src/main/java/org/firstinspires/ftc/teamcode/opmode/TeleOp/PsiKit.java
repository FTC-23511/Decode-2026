package org.firstinspires.ftc.teamcode.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.drivebase.swerve.coaxial.CoaxialSwerveModule;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;
import org.psilynx.psikit.core.rlog.RLOGServer;
import org.psilynx.psikit.core.rlog.RLOGWriter;
import org.psilynx.psikit.core.Logger;

import org.psilynx.psikit.ftc.PsiKitOpMode;

@TeleOp(name="ConceptPsiKitLogger")
public class PsiKit extends PsiKitOpMode{
        @Override
        public void runOpMode() {
            psiKitSetup();
            RLOGServer server = new RLOGServer();
            RLOGWriter writer = new RLOGWriter("logs.rlog");
            server.start();
            writer.start();
            Logger.addDataReceiver(server);
            Logger.addDataReceiver(writer);
            Logger.recordMetadata("some metadata", "string value");
            Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
            Logger.periodicAfterUser(0, 0);

            while(!getPsiKitIsStarted()){
                Logger.periodicBeforeUser();

                processHardwareInputs();
                // this MUST come before any logic

         /*

          Init logic goes here

         */

                Logger.periodicAfterUser(0.0, 0.0);
                // logging these timestamps is completely optional
            }

            // alternately the waitForStart() function works as expected.

            while(!getPsiKitIsStopRequested()) {

                double beforeUserStart = Logger.getTimestamp();
                Logger.periodicBeforeUser();
                double beforeUserEnd = Logger.getTimestamp();

                processHardwareInputs();
                // this MUST come before any logic

         /*

          OpMode logic goes here

         */

                Logger.recordOutput("OpMode/example", 2.0);
                // example


                double afterUserStart = Logger.getTimestamp();
                Logger.periodicAfterUser(
                        afterUserStart - beforeUserEnd,
                        beforeUserEnd - beforeUserStart
                );
                // alternately, keep track of how long some things are taking. up to
                // you on what you want to do
            }
            Logger.end();
        }
}
@Config
@TeleOp(name = "SwerveTeleOp")
public class SwerveOpMode extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;

    TelemetryData telemetryData = new TelemetryData(telemetry);

    private final Robot robot = Robot.getInstance();

    @Override
    public void initialize() {
        // Must have for all opModes
        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;

        // Resets the command scheduler
        super.reset();

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        // Driver controls
        // Reset heading
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> robot.drive.setPose(new Pose2d()))
        );
    }

    @Override
    public void run() {
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            robot.initHasMovement();
            timer = new ElapsedTime();
        }

        // Update any constants that are being updated by FTCDash
        for (CoaxialSwerveModule module : robot.drive.swerve.getModules()) {
            module.setSwervoPIDF(Constants.SWERVO_PIDF_COEFFICIENTS);
        }

        // Drive the robot
        double minSpeed = 0.3; // As a fraction of the max speed of the robot
        double speedMultiplier = minSpeed + (1 - minSpeed) * driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        robot.drive.swerve.updateWithTargetVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        driver.getLeftY() * Constants.MAX_VELOCITY * speedMultiplier,
                        -driver.getLeftX() * Constants.MAX_VELOCITY * speedMultiplier,
                        -driver.getRightX() * Constants.MAX_ANGULAR_VELOCITY * speedMultiplier,
                        robot.drive.getPose().getRotation()
                )
        );

        telemetryData.addData("Loop Time", timer.milliseconds());
        timer.reset();

        telemetryData.addData("Heading", robot.drive.getPose().getHeading());
        telemetryData.addData("Robot Pose", robot.drive.getPose());

        telemetryData.addData("Target Chassis Velocity", robot.drive.swerve.getTargetVelocity());
        telemetryData.addData("FR Module", robot.drive.swerve.getModules()[0].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[0].getPowerTelemetry());
        telemetryData.addData("FL Module", robot.drive.swerve.getModules()[1].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[1].getPowerTelemetry());
        telemetryData.addData("BL Module", robot.drive.swerve.getModules()[2].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[2].getPowerTelemetry());
        telemetryData.addData("BR Module", robot.drive.swerve.getModules()[3].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[3].getPowerTelemetry());

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        super.run();
        telemetryData.update();
    }

    @Override
    public void end() {
        Constants.END_POSE = robot.drive.getPose();
    }

}
