package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.globals.MathFunctions;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.Arrays;
import java.util.List;

public class Launcher extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    private final PIDFController flywheelController = new PIDFController(FLYWHEEL_PIDF_COEFFICIENTS);
    private boolean activeControl = false;
    private double targetHoodAngle = MIN_HOOD_ANGLE;
    private double targetFlywheelVelocity = 0.0;
    private boolean impossible = true;
    public static double DISTANCE_OFFSET = -0.267;

    private final List<Double> launcherInput  = Arrays.asList(-0.01, 0.0, 4.29,   4.49,   4.76,   5.22,   5.65,   6.06,   6.44,   6.86,   7.2,    10.0); // input: velocity (m/s)
    private final List<Double> launcherOutput = Arrays.asList(-0.01, 0.0, 1040.0, 1100.0, 1180.0, 1320.0, 1480.0, 1620.0, 1780.0, 1940.0, 1980.0, 2100.0); // output: ticks/s

    private final InterpLUT launcherLUT = new InterpLUT(
            launcherInput,
            launcherOutput,
            true
    );

    private final InterpLUT inverseLauncherLUT = new InterpLUT(
            launcherOutput,
            launcherInput,
            true
    );

    public Launcher() {
        launcherLUT.createLUT();
        inverseLauncherLUT.createLUT();
        flywheelController.setTolerance(FLYWHEEL_VEL_TOLERANCE);
    }

    public void init() {
        if (!TESTING_OP_MODE) {
            setRamp(OP_MODE_TYPE.equals(OpModeType.AUTO));
            setHood(MIN_HOOD_ANGLE);
        }

        setFlywheel(0, false);
    }

    public void setFlywheel(double targetVel, boolean setActiveControl) {
        flywheelController.setSetPoint(Math.min(launcherLUT.get(targetVel), LAUNCHER_MAX_VELOCITY));
        targetFlywheelVelocity = targetVel;
        setActiveControl(setActiveControl);
    }

    public void setLauncher(Pose2d robotPose) {
        double[] errorsAngleVelocity = MathFunctions.distanceToLauncherValues(GOAL_POSE().minus(robotPose).getTranslation().getNorm() * DistanceUnit.mPerInch);
        setFlywheel(errorsAngleVelocity[0], true);
        setHood(errorsAngleVelocity[1]);
    }

    /**
     * Used for INTERNAL TESTING OPMODES ONLY
     * @param vel velocity in ticks/second
     */
    public void setFlywheelTicks(double vel) {
        flywheelController.setSetPoint(vel);
        setActiveControl(true);
    }

    public double getTargetHoodAngle() {
        return targetHoodAngle;
    }

    public double getTargetFlywheelVelocity() {
        return targetFlywheelVelocity;
    }

    public void setActiveControl(boolean state) {
        activeControl = state;
        if (!state) {
            setHood(targetHoodAngle);
        }
    }

    public boolean getActiveControl() {
        return activeControl;
    }

    public double getFlywheelTarget() {
        return flywheelController.getSetPoint();
    }

    private void update() {
        robot.profiler.start("Launcher Update");
        if (activeControl) {
            double flywheelVel = robot.launchEncoder.getCorrectedVelocity();
            flywheelController.setF(FLYWHEEL_PIDF_COEFFICIENTS.f);
            robot.launchMotors.set(
                    flywheelController.calculate(flywheelVel)
            );

            if (flywheelController.atSetPoint()) {
                impossible = false;
            } else {
                // hood compensation
//                setHood(targetHoodAngle - flywheelController.getPositionError() * HOOD_BS, true);
//                impossible = true;

                double adjustedHoodAngle = MathFunctions.getHoodAngleFromVelocity(
                        GOAL_POSE().minus(robot.drive.getPose()).getTranslation().getNorm() * DistanceUnit.mPerInch,
                        inverseLauncherLUT.get(flywheelVel)
                );
                if (Double.isNaN(adjustedHoodAngle)) {
                    impossible = true;
                } else {
                    impossible = false;
                    setHood(adjustedHoodAngle, true);
                }
            }
        } else {
            if (getFlywheelTarget() == 0) {
                robot.launchMotors.set(0);
            } else {
                robot.launchMotors.set(LAUNCHER_DEFAULT_ON_SPEED);
            }

            impossible = true;
        }
        robot.profiler.end("Launcher Update");
    }

    public void setRamp(boolean engaged) {
        robot.rampServo.set(engaged ? RAMP_ENGAGED : RAMP_DISENGAGED);
    }

    public void setHood(double angle) {
        setHood(angle, false);
    }

    private void setHood(double angle, boolean compensation) {
        if (!compensation) {
            targetHoodAngle = angle;
            impossible = true;
        }

        robot.hoodServo.set(
                (angle - MIN_HOOD_ANGLE) / (MAX_HOOD_ANGLE - MIN_HOOD_ANGLE) * (MAX_HOOD_SERVO_POS - MIN_HOOD_SERVO_POS) + MIN_HOOD_SERVO_POS
        );
    }

    public boolean flywheelReady() {
        return activeControl && flywheelController.atSetPoint();
    }

    public boolean launchValid() {
        return activeControl && !impossible;
    }

    @Override
    public void periodic() {
        update();
    }
}
