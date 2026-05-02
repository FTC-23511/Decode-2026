package org.firstinspires.ftc.teamcode.tuning.drive;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.util.TelemetryEx;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp(name = "FLSwervoTuner", group = "Drive")
public class FLSwervoTuner extends CommandOpMode {
    public ElapsedTime timer;

    public static double SERVO_POWER = 0.0;

    TelemetryEx telemetryEx = new TelemetryEx(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private CRServoEx crServo;
    private AbsoluteAnalogEncoder encoder;

    @Override
    public void initialize() {
        crServo = new CRServoEx(hardwareMap, "FL");
        encoder = new AbsoluteAnalogEncoder(hardwareMap, "encoder");
    }

    @Override
    public void run() {
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            timer = new ElapsedTime();
        }

        SERVO_POWER = Range.clip(SERVO_POWER, -1.0, 1.0);
        crServo.set(SERVO_POWER);

        telemetryEx.addData("Loop Time", timer.milliseconds());
        telemetryEx.addData("Position", encoder.getCurrentPosition());
        telemetryEx.addData("Voltage", encoder.getVoltage());
        telemetryEx.addData("Raw Voltage", encoder.getEncoder().getVoltage());
        telemetryEx.update();
        timer.reset();
    }
}