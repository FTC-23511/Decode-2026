package org.firstinspires.ftc.teamcode.tuning.photon;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.TelemetryData;

import java.util.List;

@Config
@TeleOp
public class BasicPhotonTest extends LinearOpMode {
    private ElapsedTime timer = new ElapsedTime();
    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
    public static boolean ENABLE_PHOTON = true;
    public static boolean MOTOR = true;
    public static boolean SWERVO = true;
    public static double INTAKE_MOTOR_CACHE_TOL = -0.01;
    public static double SWERVO_CACHE_TOL = -0.01;
    public static double WRITES = 1;
    public static double POWER_INCREMENT = 0.0;
    double power = 0;

    @Override
    public void runOpMode() {
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        if (ENABLE_PHOTON) {
            PhotonCore.enable();
        }

        MotorEx intakeMotor = new MotorEx(hardwareMap, "intakeMotor").setCachingTolerance(INTAKE_MOTOR_CACHE_TOL);
        CRServoEx swervo = new CRServoEx(hardwareMap, "BR").setCachingTolerance(SWERVO_CACHE_TOL);

        waitForStart();

        while (opModeIsActive()) {
            timer.reset();
            intakeMotor.setCachingTolerance(INTAKE_MOTOR_CACHE_TOL);
            swervo.setCachingTolerance(SWERVO_CACHE_TOL);

            for (int i = 0; i < WRITES; i++) {
                power = power + POWER_INCREMENT;
                if (power > 1) {
                    power = -1;
                }

                if (MOTOR) {
                    intakeMotor.set(power);
                }
                if (SWERVO) {
                    swervo.set(power);
                }
            }

            telemetryData.addData("Swervo getPower", swervo.get());

            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();

            telemetryData.addData("Loop Time (ms)", timer.milliseconds());
            telemetryData.update();
        }
    }
}
