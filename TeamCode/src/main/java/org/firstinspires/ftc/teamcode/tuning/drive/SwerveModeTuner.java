package org.firstinspires.ftc.teamcode.tuning.drive;

import static org.firstinspires.ftc.teamcode.globals.Constants.TESTING_OP_MODE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.globals.Constants;

@Config
@TeleOp(name = "SwerveModeTuner", group = "Servo")
public class SwerveModeTuner extends OpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;

    public static double SERVO_POS = 0.0;
    Servo swervo;

    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    
    @Override
    public void init() {
        // Must have for all opModes
        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;
        TESTING_OP_MODE = true;

        swervo = hardwareMap.get(Servo.class, "hoodServo");

        // 2. Cast to ServoImplEx and set the range to 500-2500
        if (swervo instanceof ServoImplEx) {
            ((ServoImplEx) swervo).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
    }

    @Override
    public void loop() {
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            timer = new ElapsedTime();
        }

        SERVO_POS = Range.clip(SERVO_POS, 0.0, 1.0);
        swervo.setPosition(SERVO_POS);

        telemetryData.addData("Loop Time", timer.milliseconds());
        timer.reset();

        telemetryData.addData("SERVO_POS", SERVO_POS);

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        telemetryData.update();
    }
}