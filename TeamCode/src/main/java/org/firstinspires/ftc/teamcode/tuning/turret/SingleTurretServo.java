package org.firstinspires.ftc.teamcode.tuning.turret;

import static org.firstinspires.ftc.teamcode.globals.Constants.END_POSE;
import static org.firstinspires.ftc.teamcode.globals.Constants.TURRET_ENCODER_OFFSET;
import static org.firstinspires.ftc.teamcode.globals.Constants.TURRET_RADIANS_PER_TICK;
import static org.firstinspires.ftc.teamcode.globals.Constants.TURRET_SERVO_OFFSET;
import static org.firstinspires.ftc.teamcode.globals.Constants.TURRET_SYNC_OFFSET;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.MathUtils;
import com.seattlesolvers.solverslib.util.TelemetryEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.globals.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.globals.MathFunctions;

@Config
@TeleOp(name = "SingleTurretServo", group = "Turret")
public class SingleTurretServo extends LinearOpMode {
    private ElapsedTime timer = new ElapsedTime();
    TelemetryEx telemetryEx = new TelemetryEx(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
    public static double SERVO_POS = 0.00;
    public static boolean USE_LEFT = true;

    @Override
    public void runOpMode() {

        String id = USE_LEFT ? "leftTurretServo" : "rightTurretServo";

        ServoEx servo = new ServoEx(hardwareMap, id)
                .setCachingTolerance(0.001)
                .setInverted(true);

        AbsoluteAnalogEncoder analogTurretEncoder = new AbsoluteAnalogEncoder(hardwareMap, "turretEncoder")
                .zero(TURRET_ENCODER_OFFSET)
                .setReversed(true);

        Motor.Encoder turretEncoder = new Motor(hardwareMap, "BL").encoder
                .setDirection(Motor.Direction.FORWARD)
                .overrideResetPos((int) TURRET_SYNC_OFFSET);

        waitForStart();

        while (opModeIsActive()) {

            servo.set(SERVO_POS);
            telemetryEx.addData("Analog Encoder", MathUtils.normalizeRadians(analogTurretEncoder.getCurrentPosition(), false));
            telemetryEx.addData("Relative Encoder", MathUtils.normalizeRadians(turretEncoder.getPosition() * TURRET_RADIANS_PER_TICK, false));

            telemetryEx.addData("Loop Time (ms)", timer.milliseconds());
            telemetryEx.update();
        }
    }
}
