package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.Arrays;

/**
 * Launcher Subsystem
 * Handles the high-speed flywheel and the variable-angle hood servo.
 * Includes built-in stall protection and PIDF velocity control.
 */
public class Launcher extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    // Controller for maintaining consistent flywheel RPM regardless of battery voltage
    private final PIDFController flywheelController = new PIDFController(FLYWHEEL_PIDF_COEFFICIENTS);

    // Toggle for PID control vs. raw power/idle states
    private boolean activeControl = false;

    // Target states for the hardware
    private double targetHoodAngle = MIN_HOOD_ANGLE;
    private double targetFlywheelVelocity = 0.0;

    // --- Stall Protection Variables ---
    // Safety logic to prevent motor burnout if a game piece jams the flywheel
    private long stallStartTime = 0;
    private boolean isStalled = false;
    private static final double STALL_VELOCITY_THRESHOLD = 50.0; // Minimal movement (ticks/s)
    private static final double STALL_POWER_THRESHOLD = 0.6;    // High effort (60% power+)
    private static final long STALL_TIMEOUT_MS = 1000;           // Duration before safety trip

    public Launcher() {
        // Set how close we need to be to the target velocity to be considered "Ready"
        flywheelController.setTolerance(FLYWHEEL_VEL_TOLERANCE);
    }

    /**
     * Resets hardware to default starting positions.
     */
    public void init() {
        if (!TESTING_OP_MODE) {
            setHood(MIN_HOOD_ANGLE);
        }
        setFlywheel(0, false);
    }

    /**
     * Primary method to spin the flywheel.
     * @param vel Target velocity in encoder ticks per second.
     * @param setActiveControl If true, engages the PIDF loop.
     */
    public void setFlywheel(double vel, boolean setActiveControl) {
        if (vel > 0) isStalled = false; // Reset safety trip when a new positive command is issued
        flywheelController.setSetPoint(Math.min(vel, LAUNCHER_MAX_VELOCITY));
        targetFlywheelVelocity = vel;
        setActiveControl(setActiveControl);
    }

    /**
     * @return Current real-time velocity of the flywheel from the encoders.
     */
    public double getFlywheelVelocity() {
        return robot.launchEncoder.getCorrectedVelocity();
    }



    /**
     * Internal helper to update PID setpoints directly (used by D-Pad adjustments).
     */
    public void setFlywheelTicks(double vel) {
        flywheelController.setSetPoint(vel);
        setActiveControl(true);
    }

    /**
     * @return Current power output (-1.0 to 1.0) applied to the launcher motors.
     */
    public double getPower() {
        return robot.launchMotors.get();
    }

    public double getTargetVelocity() {
        return targetFlywheelVelocity;
    }

    public double getTargetHoodAngle() {
        return targetHoodAngle;
    }

    /**
     * @return The current target angle of the hood in degrees.
     */
    public double getHoodAngle() {
        return targetHoodAngle;
    }

    /**
     * Enables or disables the velocity PIDF loop.
     */
    public void setActiveControl(boolean state) {
        activeControl = state;
        if (state == false) {
            // Re-apply current hood angle when stopping PID to ensure servo holds position
            setHood(targetHoodAngle);
        }
    }

    public boolean getActiveControl() {
        return activeControl;
    }

    public double getFlywheelTarget() {
        return flywheelController.getSetPoint();
    }

    /**
     * Main calculation loop. Runs every ~10-20ms.
     */
    private void update() {
        robot.profiler.start("Launcher Update");

        double currentVelocity = robot.launchEncoder.getCorrectedVelocity();
        double currentPower = robot.launchMotors.get();

        // --- STALL DETECTION LOGIC ---
        // If the motor is trying hard (high power) but not moving (low velocity), start a timer.
        if (activeControl && Math.abs(currentPower) > STALL_POWER_THRESHOLD && Math.abs(currentVelocity) < STALL_VELOCITY_THRESHOLD) {
            if (stallStartTime == 0) stallStartTime = System.currentTimeMillis();
            else if (System.currentTimeMillis() - stallStartTime > STALL_TIMEOUT_MS) isStalled = true;
        } else {
            stallStartTime = 0;
        }

        // Emergency shutdown if stalled
        if (isStalled) {
            robot.launchMotors.set(0);
            activeControl = false;
        }
        // Normal PID operation
        else if (activeControl) {
            flywheelController.setPIDF(
                    FLYWHEEL_PIDF_COEFFICIENTS.p,
                    FLYWHEEL_PIDF_COEFFICIENTS.i,
                    FLYWHEEL_PIDF_COEFFICIENTS.d,
                    FLYWHEEL_PIDF_COEFFICIENTS.f
            );

            robot.launchMotors.set(flywheelController.calculate(currentVelocity));
            setHood(targetHoodAngle, false); // Constantly update servo to fight vibrations
        }
        // Idle/Manual state
        else {
            // Apply a default "on" speed if targeted, otherwise 0
            robot.launchMotors.set(targetFlywheelVelocity == 0 ? 0 : LAUNCHER_DEFAULT_ON_SPEED);
        }

        robot.profiler.end("Launcher Update");
    }

    /**
     * Sets the target angle of the hood.
     * @param angle Angle in degrees (e.g., 10 to 45).
     */
    public void setHood(double angle) {
        setHood(angle, false);
    }

    /**
     * Internal hood setter with safety clipping and servo mapping.
     */
    private void setHood(double angle, boolean compensation) {
        // Prevent servo from driving past physical hard stops
        double angle2 = Range.clip(angle, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE);

        if (!compensation) {
            targetHoodAngle = angle2;
        }

        // LERP Mapping: Converts Degrees (10-45) to Servo Position (0.0-1.0)
        // Formula: (Target - Min) / (Range) * (ServoRange) + MinServo
        robot.hoodServo.set(
                (angle2 - MIN_HOOD_ANGLE) / (MAX_HOOD_ANGLE - MIN_HOOD_ANGLE) * (MAX_HOOD_SERVO_POS - MIN_HOOD_SERVO_POS) + MIN_HOOD_SERVO_POS
        );
    }

    public boolean isStalled() {
        return isStalled;
    }

    // --- MANUAL D-PAD ADJUSTMENT METHODS ---

    public void adjustHoodDown() {
        setHood(targetHoodAngle - 2.0); // Decrease by 2 degrees
    }

    public void adjustHoodUp() {
        setHood(targetHoodAngle + 2.0); // Increase by 2 degrees
    }

    public void adjustFlywheelSpeedUp() {
        setFlywheelTicks(flywheelController.getSetPoint() + 100.0); // Increase by 100 ticks/s
    }

    public void adjustFlywheelSpeedDown() {
        setFlywheelTicks(flywheelController.getSetPoint() - 100.0); // Decrease by 100 ticks/s
    }

    /**
     * @return True if the flywheel is within the allowed velocity tolerance of the target.
     */
    public boolean flywheelReady() {
        return activeControl && flywheelController.atSetPoint();
    }

    @Override
    public void periodic() {
        update(); // Subsystem requirement: logic must run in periodic()
    }
}
