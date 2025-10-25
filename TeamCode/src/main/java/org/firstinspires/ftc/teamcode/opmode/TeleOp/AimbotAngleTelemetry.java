package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Disabled
@TeleOp
public class AimbotAngleTelemetry extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;

    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();
    final double gravity = 9.81;
    final double launcherReleaseHeight = 0.38;
    final double basketHeight = 1.45;
    final double exitVelo = 8.0;

    final double TARGET_X = -1.7018; //-67 inches btw
    final double TARGET_Y = 1.7018; //67 inches btw

    final double launcher_DX = 0.00;
    final double launcher_DY = 0.00;

    double dx, dy;        // delta X, delta Y (m)
    double d;             // horizontal distance (m)
    double dH;            // vertical delta (m)
    double v_mps;         // velocity (m/s)
    double thetaRad;      // hood angle (rad)

    double v_ips = v_mps / 0.0254;   // convert m/s → in/s



    public void initialize() {

    }
}