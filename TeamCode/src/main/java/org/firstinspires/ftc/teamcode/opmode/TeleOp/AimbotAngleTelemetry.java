package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Transform2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Disabled
@TeleOp(name = "Aimbot Telemetry")
public class AimbotAngleTelemetry extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;

    TelemetryData telemetryData = new TelemetryData(telemetry);

    final double gravity = 9.81;
    final double shooterReleaseHeight = 0.38;
    final double basketHeight = 1.0414;
    public static double exitVelo = 8.0;

    final double TARGET_X = -1.7018; //-67 inches btw
    final double TARGET_Y = 1.7018; //67 inches btw

    final double SHOOTER_DX = 0.00;
    final double SHOOTER_DY = 0.00;



    private final Robot robot = Robot.getInstance();


    public void initialize() {
        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;


        super.reset();

        robot.init(hardwareMap);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

    }

    public void run(){
        // Shooter offset transform
        Transform2d shooterOffset = new Transform2d(
                new Translation2d(SHOOTER_DX, SHOOTER_DY),
                new Rotation2d(0.0)
        );

        double dx = TARGET_X - (robot.drive.getPose().plus(shooterOffset).getX());
        double dy = TARGET_Y - (robot.drive.getPose().plus(shooterOffset).getY());
        double d  = Math.hypot(dx, dy);

        double dH = basketHeight - shooterReleaseHeight;

        double v2 = exitVelo * exitVelo;
        double disc = v2*v2 - gravity * (gravity * d * d + 2.0 * dH * v2);

        Double thetaLowRad = null, thetaHighRad = null;
        if (disc >= 0.0 && d > 1e-6) {
            double root    = Math.sqrt(disc);
            double denom   = gravity * d;
            double tanHigh = (v2 + root) / denom;
            double tanLow  = (v2 - root) / denom;

            thetaHighRad = Math.atan(tanHigh);
            thetaLowRad  = Math.atan(tanLow);


        }
        double v_ips = exitVelo / 0.0254;  // m/s → in/s

        telemetryData.addData("Loop Time", timer.milliseconds());
        telemetryData.addData("Heading", robot.drive.getPose().getHeading());

        telemetryData.addData("Robot Pose (x,y,θ°)",
                robot.drive.getPose().getX() + ", " +
                        robot.drive.getPose().getY() + ", " +
                        Math.toDegrees(robot.drive.getPose().getRotation().getRadians()));

        telemetryData.addData("Shooter Pos (x_s,y_s)",
                robot.drive.getPose().plus(shooterOffset).getX() + ", " +
                        robot.drive.getPose().plus(shooterOffset).getY());

        telemetryData.addData("Distance (in)", d / 0.0254);
        telemetryData.addData("Δh (in)", dH / 0.0254);
        telemetryData.addData("Velocity (in/s)", v_ips);

        if (thetaLowRad != null) {
            telemetryData.addData("θ_low (rad)", thetaLowRad);
            telemetryData.addData("θ_high (rad)", thetaHighRad);
        } else {
            telemetryData.addData("θ", "NO REAL SOLUTION");
        }

        telemetryData.update();


    }
}