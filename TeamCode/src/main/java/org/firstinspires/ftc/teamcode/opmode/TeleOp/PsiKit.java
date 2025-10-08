package org.firstinspires.ftc.teamcode.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;
import org.psilynx.psikit.core.rlog.RLOGServer;
import org.psilynx.psikit.core.rlog.RLOGWriter;
import org.psilynx.psikit.core.Logger;

import org.psilynx.psikit.ftc.PsiKitOpMode;

@TeleOp(name="ConceptPsiKitLogger")
public class PsiKit extends PsiKitOpMode{
        public GamepadEx driver;
        public GamepadEx operator;
        public ElapsedTime timer;
        TelemetryData telemetryData = new TelemetryData(telemetry);

    private final Robot robot = Robot.getInstance();
        public void initialize() {
            // Must have for all opModes
            Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;

            // Resets the command scheduler
            CommandScheduler.getInstance().reset();

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
