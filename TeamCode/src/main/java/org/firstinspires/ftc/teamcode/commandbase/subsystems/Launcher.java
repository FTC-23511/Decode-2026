package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.globals.MathFunctions;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.Arrays;

public class Launcher extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    private final PIDFController flywheelController = new PIDFController(FLYWHEEL_PIDF_COEFFICIENTS);
    private boolean activeControl = false;
    private double targetHoodAngle = MIN_HOOD_ANGLE;
    private double targetFlywheelVelocity = 0.0;
    private boolean impossible = false;

    private final InterpLUT launcherLUT = new InterpLUT(
            Arrays.asList(-0.01, 0.0, 4.29,   4.76,   5.22,   5.65,   6.06,   6.48,   10.0), // input: velocity (m/s)
            Arrays.asList(0.0,   0.0, 1267.0, 1367.0, 1500.0, 1667.0, 1790.0, 1967.0, 2000.0), // output: ticks/s
            true
    );

    public Launcher() {
        launcherLUT.createLUT();
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
            flywheelController.setF(FLYWHEEL_PIDF_COEFFICIENTS.f);
            robot.launchMotors.set(
                    flywheelController.calculate(robot.launchEncoder.getCorrectedVelocity())
            );

//            setHood(targetHoodAngle, true);
        } else {
            if (getFlywheelTarget() == 0) {
                robot.launchMotors.set(0);
            } else {
                robot.launchMotors.set(LAUNCHER_DEFAULT_ON_SPEED);
            }

            impossible = false;
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
        if (compensation) {
            double launchVel = robot.launchEncoder.getCorrectedVelocity();
            robot.launchMotors.set(flywheelController.calculate(launchVel));

            Object[] newHoodAngle = MathFunctions.getHoodAngleFromVelocity(
                    GOAL_POSE().minus(robot.drive.getPose()).getTranslation().getNorm() * DistanceUnit.mPerInch,
                    MathFunctions.convertToMetersPerSec(launchVel)
            );

            angle = (double) newHoodAngle[0];
            impossible = (boolean) newHoodAngle[1];
        }

//        robot.hoodServo.set(Range.clip(angle, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE));
        robot.hoodServo.set(
                (angle - MIN_HOOD_ANGLE) / (MAX_HOOD_ANGLE - MIN_HOOD_ANGLE) * (MAX_HOOD_SERVO_POS - MIN_HOOD_SERVO_POS) + MIN_HOOD_SERVO_POS
        );
    }

    public boolean flywheelReady() {
        return activeControl && flywheelController.atSetPoint();
    }

    public boolean hoodReady() {
        return activeControl && robot.hoodServo.atSetPosition() && !impossible;
    }

    @Override
    public void periodic() {
        update();
    }
}
