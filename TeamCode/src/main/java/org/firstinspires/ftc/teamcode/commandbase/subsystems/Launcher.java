package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.Arrays;

public class Launcher extends SubsystemBase {
    private final Robot robot = Robot.getInstance();


    private final PIDFController flywheelController = new PIDFController(FLYWHEEL_PIDF_COEFFICIENTS);
    private boolean activeControl = false;
    private double targetHoodAngle = MIN_HOOD_ANGLE;
    private double targetFlywheelVelocity = 0.0;

    // Stall Protection Variables
    private long stallStartTime = 0;
    private boolean isStalled = false;
    private static final double STALL_VELOCITY_THRESHOLD = 50.0; // ticks/s (very low movement)
    private static final double STALL_POWER_THRESHOLD = 0.6;    // 60% power or higher
    private static final long STALL_TIMEOUT_MS = 2500;           // 2.5 seconds before cutting power



    public Launcher() {
        flywheelController.setTolerance(FLYWHEEL_VEL_TOLERANCE);
    }

    public void init() {
        if (!TESTING_OP_MODE) {
            setHood(MIN_HOOD_ANGLE);
        }
        setFlywheel(0, false);
    }

    public void setFlywheel(double vel, boolean setActiveControl) {
        if (vel > 0) isStalled = false; // Reset stall state when a new command is sent
        flywheelController.setSetPoint(Math.min(vel, LAUNCHER_MAX_VELOCITY));
        targetFlywheelVelocity = vel;
        setActiveControl(setActiveControl);
    }

    /**
     * @return the current actual velocity of the flywheel in ticks/second.
     */
    public double getFlywheelVelocity() {
        return robot.launchEncoder.getCorrectedVelocity();
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

    /**
     * @return the current target hood angle in degrees.
     */
    public double getHoodAngle() {
        return targetHoodAngle;
    }


    public void setActiveControl(boolean state) {
        activeControl = state;
        if (state == false) {
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

        double currentVelocity = robot.launchEncoder.getCorrectedVelocity();
        double currentPower = robot.launchMotors.get(); // Assuming your motor wrapper has get()

        // --- Stall Detection Logic ---
        if (activeControl && Math.abs(currentPower) > STALL_POWER_THRESHOLD && Math.abs(currentVelocity) < STALL_VELOCITY_THRESHOLD) {
            if (stallStartTime == 0) {
                stallStartTime = System.currentTimeMillis();
            } else if (System.currentTimeMillis() - stallStartTime > STALL_TIMEOUT_MS) {
                isStalled = true;
            }
        } else {
            stallStartTime = 0;
            // Note: We don't automatically set isStalled to false here
            // so the driver has to manually reset the state.
        }

        if (isStalled) {
            robot.launchMotors.set(0);
            activeControl = false;
            // Optional: Log a warning or send telemetry
        } else if (activeControl) {
            flywheelController.setF(FLYWHEEL_PIDF_COEFFICIENTS.f);
            robot.launchMotors.set(
                    flywheelController.calculate(currentVelocity)
            );

            setHood(targetHoodAngle, false);
        } else {
            if (getFlywheelTarget() == 0) {
                robot.launchMotors.set(0);
            } else {
                robot.launchMotors.set(LAUNCHER_DEFAULT_ON_SPEED);
            }
        }
        robot.profiler.end("Launcher Update");
    }



    public void setHood(double angle) {
        setHood(angle, false);
    }

    private void setHood(double angle, boolean compensation) {
        double angle2 = Range.clip(angle, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE);
        if (!compensation) {
            targetHoodAngle = angle2;
        }
        // Solved from proportion (targetServo - minServo) / servoRange = (targetAngle - minAngle) / angleRange
        robot.hoodServo.set(
                (angle2 - MIN_HOOD_ANGLE) / (MAX_HOOD_ANGLE - MIN_HOOD_ANGLE) * (MAX_HOOD_SERVO_POS - MIN_HOOD_SERVO_POS) + MIN_HOOD_SERVO_POS
        );
    }

    public boolean isStalled() {
        return isStalled;
    }

    /**
     * Decreases the hood angle by 2 degrees.
     */
    public void adjustHoodDown() {
        setHood(targetHoodAngle - 2.0);
    }

    /**
     * Increases the hood angle by 2 degrees.
     */
    public void adjustHoodUp() {
        setHood(targetHoodAngle + 2.0);
    }

    /**
     * Increases the flywheel target velocity by 100 ticks/s.
     */
    public void adjustFlywheelSpeedUp() {
        setFlywheelTicks(flywheelController.getSetPoint() + 100.0);
    }

    /**
     * Decreases the flywheel target velocity by 100 ticks/s.
     */
    public void adjustFlywheelSpeedDown() {
        setFlywheelTicks(flywheelController.getSetPoint() - 100.0);
    }



    public boolean flywheelReady() {
        return activeControl && flywheelController.atSetPoint();
    }


    @Override
    public void periodic() {
        update();
    }
}
