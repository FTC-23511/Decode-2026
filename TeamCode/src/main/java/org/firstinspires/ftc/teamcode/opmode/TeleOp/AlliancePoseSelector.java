package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class AlliancePoseSelector extends LinearOpMode {
    private ElapsedTime buttonTimer;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Cross / Triangle", "Blue");
        telemetry.addData("Circle / Square", "Red");
        telemetry.addData("Alliance Color", ALLIANCE_COLOR);

        telemetry.update();

        waitForStart();
        buttonTimer = new ElapsedTime();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.cross || gamepad2.cross || gamepad1.triangle || gamepad2.triangle) {
                ALLIANCE_COLOR = AllianceColor.BLUE;
            } else if (gamepad1.circle || gamepad2.circle || gamepad1.square || gamepad2.square) {
                ALLIANCE_COLOR = AllianceColor.RED;
            }

            telemetry.addData("Cross / Triangle", "Blue");
            telemetry.addData("Circle / Square", "Red");
            telemetry.addData("Alliance Color", ALLIANCE_COLOR);

            telemetry.update();
        }
    }
}