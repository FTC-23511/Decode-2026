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

    public enum Motif {
        NOT_FOUND,
        GPP,
        PGP,
        PPG
    }

    public static Motif motifState = Motif.NOT_FOUND;
    private final PIDFController flywheelController = new PIDFController(FLYWHEEL_PIDF_COEFFICIENTS);
    private boolean activeControl = false;
    private double targetHoodAngle = MIN_HOOD_ANGLE;
    private double targetFlywheelVelocity = 0.0;

    private final InterpLUT launcherVel = new InterpLUT(
            Arrays.asList(-0.01, 0.0, 4.29,   4.76,   5.22,   5.65,   6.06,   6.48,   10.0), // input: velocity (m/s)
            Arrays.asList(0.0,   0.0, 1267.0, 1367.0, 1500.0, 1667.0, 1790.0, 1967.0, 2000.0) // output:
    );

    public Launcher() {
        launcherVel.createLUT();
        flywheelController.setTolerance(FLYWHEEL_VEL_TOLERANCE);
    }

    public void init() {
        setRamp(OP_MODE_TYPE == OpModeType.AUTO);
        setHood(MIN_HOOD_ANGLE);
        setFlywheel(0, false);
    }

    public void setFlywheel(double vel, boolean setActiveControl) {
        flywheelController.setSetPoint(Math.min(launcherVel.get(vel), LAUNCHER_MAX_VELOCITY));
        targetFlywheelVelocity = vel;
        activeControl = setActiveControl;
    }

    /**
     * Used for INTERNAL TESTING OPMODES ONLY
     * @param vel velocity in ticks/second
     */
    public void setFlywheelTicks(double vel) {
        flywheelController.setSetPoint(vel);
        activeControl = true;
    }

    public double getTargetHoodAngle() {
        return targetHoodAngle;
    }

    public double getTargetFlywheelVelocity() {
        return targetFlywheelVelocity;
    }

    public void setActiveControl(boolean state) {
        activeControl = state;
    }

    public boolean getActiveControl() {
        return activeControl;
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
        double angle2 = Range.clip(angle, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE);
        targetHoodAngle = angle2;
        // Solved from proportion (targetServo - minServo) / servoRange = (targetAngle - minAngle) / angleRange
        robot.hoodServo.set(
                (angle2 - MIN_HOOD_ANGLE) / (MAX_HOOD_ANGLE - MIN_HOOD_ANGLE) * (MAX_HOOD_SERVO_POS - MIN_HOOD_SERVO_POS) + MIN_HOOD_SERVO_POS
        );
    }

    public boolean flywheelReady() {
        return activeControl && flywheelController.atSetPoint();
    }

    /**
     * Calculates the required launch parameters (velocity and angle) that
     * use the lowest possible velocity while respecting ALL constraints.
     *
     * @param distance The horizontal distance to the target (x) in meters.
     * @return A 2-item List<Double> containing:
     * - Index 0: required velocity (m/s)
     * - Index 1: used launch angle (degrees) measured FROM THE VERTICAL.
     * Returns [Double.NaN, Double.NaN] if the shot is impossible.
     */
    public static double[] distanceToLauncherValues(double distance) {
        double g = GRAVITY;
        double x = distance;
        double deltaY = TARGET_HEIGHT - LAUNCHER_HEIGHT;

        // --- 1. Calculate the theoretical minimum velocity shot ---

        // v²_min = g * (Δy + sqrt(Δy² + x²))
        double minVelocitySquared = g * (deltaY + Math.sqrt(Math.pow(deltaY, 2) + Math.pow(x, 2)));
        double minVelocity = Math.sqrt(minVelocitySquared);

        // Calculate the angle required for this absolute minimum velocity (FROM HORIZONTAL)
        double tanThetaMin = minVelocitySquared / (g * x);
        double optimalAngleHoriz = Math.toDegrees(Math.atan(tanThetaMin));

        // Convert optimal angle to the vertical system for checking constraints
        double optimalAngleVert = 90.0 - optimalAngleHoriz;


        // --- 2. Determine the Final Angle (Vertical) for the solution ---

        double finalAngleVert; // The angle we will use and return (FROM VERTICAL)
        double finalAngleHoriz; // The angle used in the physics calculation (FROM HORIZONTAL)

        if (optimalAngleVert >= MIN_HOOD_ANGLE && optimalAngleVert <= MAX_HOOD_ANGLE) {
            // Case A: Optimal shot is within vertical angle limits. Use it.
            finalAngleVert = optimalAngleVert;
            finalAngleHoriz = optimalAngleHoriz;

            // Check velocity limit for this optimal shot
            if (minVelocity > MAX_DRIVE_VELOCITY) {
                // Even the most efficient shot is too fast. IMPOSSIBLE.
                return new double[]{Double.NaN, Double.NaN};
            }

            // Return the optimal, efficient solution
            return new double[]{minVelocity, finalAngleVert};

        } else if (optimalAngleVert < MIN_HOOD_ANGLE) {
            // Case B: Optimal angle is too close to vertical. FORCED to MIN_ANGLE_VERT (16°).
            finalAngleVert = MIN_HOOD_ANGLE;
            finalAngleHoriz = 90.0 - MIN_HOOD_ANGLE; // Convert to horizontal system (90-16=74 deg)

        } else { // optimalAngleVert > MAX_ANGLE_VERT
            // Case C: Optimal angle is too close to horizontal. FORCED to MAX_ANGLE_VERT (50°).
            finalAngleVert = MAX_HOOD_ANGLE;
            finalAngleHoriz = 90.0 - MAX_HOOD_ANGLE; // Convert to horizontal system (90-50=40 deg)
        }

        if (distance <= 0.825) {
            finalAngleVert = MIN_HOOD_ANGLE;
            finalAngleHoriz = 90.0 - MIN_HOOD_ANGLE; // Convert to horizontal system (90-50=40 deg)
        }

        // --- 3. Recalculate Velocity for the Forced Angle (Cases B and C) ---

        double angleToUseRad = Math.toRadians(finalAngleHoriz);
        double tanTheta = Math.tan(angleToUseRad);
        double cosTheta = Math.cos(angleToUseRad);

        // v₀² = (g * x²) / (2 * cos²(θ) * (x * tan(θ) - Δy))
        double denominator = 2 * (cosTheta * cosTheta) * (x * tanTheta - deltaY);

        // Check for physical impossibility (denominator <= 0)
        if (denominator <= 0) {
            return new double[]{Double.NaN, Double.NaN};
        }

        double requiredVelocity = Math.sqrt((g * x * x) / denominator);

        // --- 4. Final Velocity Constraint Check and Return ---

        if (requiredVelocity > MAX_DRIVE_VELOCITY) {
            // The required velocity for the forced angle is too high. IMPOSSIBLE.
            return new double[]{Double.NaN, Double.NaN};
        }

        // Return the valid, constrained solution
        return new double[]{requiredVelocity, finalAngleVert};
    }

    @Override
    public void periodic() {
        updateFlywheel();
    }
}
