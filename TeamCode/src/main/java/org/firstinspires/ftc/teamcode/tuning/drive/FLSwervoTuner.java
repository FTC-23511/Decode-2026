package org.firstinspires.ftc.teamcode.tuning.drive;

import static org.firstinspires.ftc.teamcode.globals.Constants.TESTING_OP_MODE;

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
import com.seattlesolvers.solverslib.util.TelemetryEx;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp(name = "FLSwervoTuner", group = "Drive")
public class FLSwervoTuner extends CommandOpMode {
    public ElapsedTime timer;

    public static double SERVO_POWER = 0.0;

    TelemetryEx telemetryEx = new TelemetryEx(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private com.qualcomm.robotcore.hardware.CRServo crServo;

    @Override
    public void initialize() {
        crServo = hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class, "FL");
    }

    @Override
    public void run() {
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            timer = new ElapsedTime();
        }

        SERVO_POWER = Range.clip(SERVO_POWER, 0.0, 1.0);
        crServo.setPower(SERVO_POWER);

        telemetryEx.addData("Loop Time", timer.milliseconds());
        timer.reset();


        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
    }
}