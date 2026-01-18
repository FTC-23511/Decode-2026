package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.globals.Robot;

@TeleOp(group = "AAATeleOp")
public class AlliancePoseSelector extends LinearOpMode {
//    private ElapsedTime buttonTimer;
    private final Robot robot = Robot.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Cross / Triangle", "Blue");
        telemetry.addData("Circle / Square", "Red");
        telemetry.addData("Alliance Color", ALLIANCE_COLOR);

        telemetry.update();
        OP_MODE_TYPE = OpModeType.TELEOP;
        TESTING_OP_MODE = true;
        robot.init(hardwareMap);

        robot.turretServos.set(0.01);

        waitForStart();
//        buttonTimer = new ElapsedTime();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.cross || gamepad2.cross || gamepad1.triangle || gamepad2.triangle) {
                ALLIANCE_COLOR = AllianceColor.BLUE;
            } else if (gamepad1.circle || gamepad2.circle || gamepad1.square || gamepad2.square) {
                ALLIANCE_COLOR = AllianceColor.RED;
            }

            robot.turretServos.set(0.01);

            if (gamepad1.right_stick_button) {
                TURRET_SYNCED = false;
                robot.turret.resetTurretEncoder();
            }

            telemetry.addData("Cross / Triangle", "Blue");
            telemetry.addData("Circle / Square", "Red");
            telemetry.addData("Alliance Color", ALLIANCE_COLOR);

            telemetry.addData("Encoder Pos", robot.turret.getPosition());;
            telemetry.addData("Analog Pos", MathUtils.normalizeRadians(robot.analogTurretEncoder.getCurrentPosition(), false));

            telemetry.update();
            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();
        }

        robot.turretServos.set(0.0);
    }
}