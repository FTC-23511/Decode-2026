package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.globals.Robot;

public class Launcher extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    public boolean activeControl = false;

    private final PIDFController flywheelController = new PIDFController(FLYWHEEL_PIDF_COEFFICIENTS);

    public void init() {
        setRamp(OP_MODE_TYPE == OpModeType.AUTO);
        setHood(MIN_HOOD_ANGLE);
        setFlywheel(0, false);
    }

    public void setFlywheel(double vel, boolean setActiveControl) {
        flywheelController.setSetPoint(vel * M_S_TO_TICKS);
        activeControl = setActiveControl;
    }

    public void setActiveControl(boolean state) {
        activeControl = state;
    }

    public double getFlywheelTarget() {
        return flywheelController.getSetPoint();
    }

    private void updateFlywheel() {
        robot.profiler.start("Launcher Update");
        if (activeControl) {
            flywheelController.setF(FLYWHEEL_PIDF_COEFFICIENTS.f / (robot.getVoltage() / 12));
            robot.launchMotors.set(
                    flywheelController.calculate(robot.launchEncoder.getCorrectedVelocity())
            );
        } else {
            if (getFlywheelTarget() == 0) {
                robot.launchMotors.set(0);
            } else {
                robot.launchMotors.set(LAUNCHER_DEFAULT_ON_SPEED);
            }
        }
        robot.profiler.end("Launcher Update");
    }

    public void setRamp(boolean engaged) {
        robot.rampServo.set(engaged ? RAMP_ENGAGED : RAMP_DISENGAGED);
    }

    public void setHood(double angle) {
        // Solved from proportion (targetServo - minServo) / servoRange = (targetAngle - minAngle) / angleRange
        robot.hoodServo.set(
                (angle - MIN_HOOD_ANGLE) / (MAX_HOOD_ANGLE - MIN_HOOD_ANGLE) * (MAX_HOOD_SERVO_POS - MIN_HOOD_SERVO_POS) + MIN_HOOD_SERVO_POS
        );
    }

    public boolean flywheelReady() {
        return activeControl && flywheelController.atSetPoint();
    }

    @Override
    public void periodic() {
        updateFlywheel();
    }
}
