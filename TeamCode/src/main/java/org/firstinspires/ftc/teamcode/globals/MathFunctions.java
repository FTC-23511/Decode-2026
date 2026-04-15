package org.firstinspires.ftc.teamcode.globals;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Launcher;
import org.opencv.core.Point;

public class MathFunctions {
    /**
     * Converts pixels on a camera to degrees, give its FX, FY, CX, and CY
     */
    public static double[] pixelsToDegrees(Point center) {
        double px = center.x;
        double py = center.y;

        double xNorm = (px - 317.108) / 549.651;
        double yNorm = (py - 236.644) / 549.651;

        double horizontalAngle = Math.toDegrees(Math.atan(xNorm));
        double verticalAngle = Math.toDegrees(Math.atan(yNorm));

        return new double[]{horizontalAngle, verticalAngle};
    }

    /**
     * Gets height and width of a rectangle defined by 4 Points (corners)
     */
    public static double[] getPointDimensions(Point[] corners) {
        if (corners == null) {
            return new double[]{-1, -1};
        }

        double minY = Double.MAX_VALUE;
        double maxY = Double.MIN_VALUE;
        double minX = Double.MAX_VALUE;
        double maxX = Double.MIN_VALUE;

        for (Point point : corners) {
            if (point.y < minY) {
                minY = point.y;
            }

            if (point.y > maxY) {
                maxY = point.y;
            }

            if (point.x < minX) {
                minX = point.x;
            }

            if (point.x > maxX) {
                maxX = point.x;
            }
        }

        return new double[]{maxX - minX, maxY - minY};
    }

    /**
     * Standard Linear Equation Solver (Point-Slope Form)
     * Calculates y based on x, given two known points (x1, y1) and (x2, y2).
     * <p>
     * y - y1 = m(x - x1)
     * -> y = m(x - x1) + y1
     */
    public static double mapEquation(double x, double x1, double y1, double x2, double y2) {
        // Clamps input
        if (x <= x1) {
            return y1;
        }
        if (x >= x2) {
            return y2;
        }

        double m = (y2 - y1) / (x2 - x1);

        return m * (x - x1) + y1;
    }

    public static double convertRadianToServoPos(double radians) {
        if (Double.isNaN(radians)) {
            return Double.NaN;
        }

        return (radians + (Math.toRadians(TURRET_SERVO_ROTATION) / 2.0)) / Math.toRadians(TURRET_SERVO_ROTATION);
    }

    public static double convertServoPosToRadian(double pos) {
        if (Double.isNaN(pos)) {
            return Double.NaN;
        }

        return (pos * Math.toRadians(TURRET_SERVO_ROTATION)) - (Math.toRadians(TURRET_SERVO_ROTATION) / 2.0);
    }

    public static double[] distanceToLauncherValues(double distance) {
        return distanceToLauncherValues(distance, Launcher.preferredHoodAngleLUT.get(distance));
    }

    /**
     * Calculates the best launch parameters (Velocity, Angle) for a given distance.
     * @param distance Horizontal distance to goal center (m).
     * @param preferredAngleV Launch angle from vertical (deg). If NaN, defaults to flattest shot.
     * @return { velocity (m/s), angleFromVertical (deg) }
     */
    public static double[] distanceToLauncherValues(double distance, double preferredAngleV) {
        distance += Launcher.DISTANCE_OFFSET;
        double g = GRAVITY;
        double x = distance;
        double xLip = x - GOAL_LIP;
        double deltaYLip = (TARGET_HEIGHT + LIP_BUFFER) - LAUNCHER_HEIGHT;

        // --- Attempt 1: Backboard Shot ---
        double[] result = calculateBestShot(x, TARGET_HEIGHT + BACKBOARD_Y_OFFSET, xLip, deltaYLip, g, preferredAngleV);

        // --- Attempt 2: Center Goal Fallback ---
        if (Double.isNaN(result[0])) {
            result = calculateBestShot(x, TARGET_HEIGHT, xLip, deltaYLip, g, preferredAngleV);
        }

        return result;
    }

    private static double[] calculateBestShot(double x, double targetY, double xLip, double deltaYLip, double g, double preferredAngleV) {
        double deltaY = targetY - LAUNCHER_HEIGHT;
        double maxPhysAngleH = 90.0 - MIN_HOOD_ANGLE;
        double minPhysAngleH = 90.0 - MAX_HOOD_ANGLE;

        // 1. Minimum Angle Constraints (Lip & Line-of-Sight)
        double minGeomAngle = Math.toDegrees(Math.atan(deltaY / x)) + 0.1;
        double minLipH = 0.0;
        if (xLip > 0) {
            double num = (deltaY * xLip * xLip) - (deltaYLip * x * x);
            double den = (x * xLip * xLip) - (xLip * x * x);
            minLipH = (Math.abs(den) > 1e-5) ? Math.toDegrees(Math.atan(num / den)) : 89.9;
        }

        double absoluteMinAngleH = Math.max(minPhysAngleH, Math.max(minLipH, minGeomAngle));

        // 2. Determine Target Angle based on NaN check
        double targetAngleH;
        if (Double.isNaN(preferredAngleV)) {
            targetAngleH = absoluteMinAngleH; // Fallback to Flattest
        } else {
            targetAngleH = Math.max(90.0 - preferredAngleV, absoluteMinAngleH); // Use Preferred
        }

        // 3. Try Velocity for Target Angle
        if (targetAngleH <= maxPhysAngleH) {
            double vReq = calculateVelocity(x, deltaY, targetAngleH, g);
            if (!Double.isNaN(vReq) && vReq <= LAUNCHER_MAX_BALL_VELOCITY) {
                return new double[]{vReq, 90.0 - targetAngleH};
            }
        }

        // 4. Maximum Power Fallback
        double v = LAUNCHER_MAX_BALL_VELOCITY;
        double A = (g * x * x) / (2.0 * v * v);
        double B = -x;
        double C = deltaY + A;
        double disc = B * B - 4 * A * C;

        if (disc < 0) return new double[]{Double.NaN, Double.NaN};

        double sqrtD = Math.sqrt(disc);
        double a1 = Math.toDegrees(Math.atan((-B - sqrtD) / (2 * A))); // Flatter
        double a2 = Math.toDegrees(Math.atan((-B + sqrtD) / (2 * A))); // Steeper

        boolean a1Valid = a1 >= absoluteMinAngleH && a1 <= maxPhysAngleH;
        boolean a2Valid = a2 >= absoluteMinAngleH && a2 <= maxPhysAngleH;

        // If Preferred is NaN, always take the flattest valid shot (a1)
        if (Double.isNaN(preferredAngleV)) {
            if (a1Valid) return new double[]{v, 90.0 - a1};
            if (a2Valid) return new double[]{v, 90.0 - a2};
        } else {
            // Otherwise, pick the root closest to the preference
            double preferredH = 90.0 - preferredAngleV;
            if (a1Valid && a2Valid) {
                return (Math.abs(a1 - preferredH) < Math.abs(a2 - preferredH))
                        ? new double[]{v, 90.0 - a1} : new double[]{v, 90.0 - a2};
            }
            if (a1Valid) return new double[]{v, 90.0 - a1};
            if (a2Valid) return new double[]{v, 90.0 - a2};
        }

        return new double[]{Double.NaN, Double.NaN};
    }

    public static double getHoodAngleFromVelocity(double distance, double velocity, double preferredAngleV) {
        double g = GRAVITY;
        double x = distance;
        double xLip = x - GOAL_LIP;
        double deltaYLip = (TARGET_HEIGHT + LIP_BUFFER) - LAUNCHER_HEIGHT;

        double angle = solveForTarget(x, TARGET_HEIGHT + BACKBOARD_Y_OFFSET, xLip, deltaYLip, velocity, g, preferredAngleV);
        if (Double.isNaN(angle)) {
            angle = solveForTarget(x, TARGET_HEIGHT, xLip, deltaYLip, velocity, g, preferredAngleV);
        }
        return angle;
    }

    private static double solveForTarget(double x, double targetY, double xLip, double deltaYLip, double v, double g, double preferredAngleV) {
        double deltaY = targetY - LAUNCHER_HEIGHT;
        double A = (g * x * x) / (2.0 * v * v);
        double B = -x;
        double C = deltaY + A;

        double disc = (B * B) - (4.0 * A * C);
        if (disc < 0) return Double.NaN;

        double sqrtD = Math.sqrt(disc);
        double a1H = Math.toDegrees(Math.atan((-B - sqrtD) / (2.0 * A)));
        double a2H = Math.toDegrees(Math.atan((-B + sqrtD) / (2.0 * A)));

        double minLipH = 0.0;
        if (xLip > 0) {
            double num = (deltaY * xLip * xLip) - (deltaYLip * x * x);
            double den = (x * xLip * xLip) - (xLip * x * x);
            minLipH = Math.toDegrees(Math.atan(num / den));
        }

        boolean a1Valid = isAngleValid(a1H, minLipH, 1e-7);
        boolean a2Valid = isAngleValid(a2H, minLipH, 1e-7);

        // NaN Check for Hood Compensation
        if (Double.isNaN(preferredAngleV)) {
            if (a1Valid) return 90.0 - a1H; // Default to flattest
        } else {
            double prefH = 90.0 - preferredAngleV;
            if (a1Valid && a2Valid) {
                return (Math.abs(a1H - prefH) < Math.abs(a2H - prefH)) ? 90.0 - a1H : 90.0 - a2H;
            }
            if (a1Valid) return 90.0 - a1H;
            if (a2Valid) return 90.0 - a2H;
        }

        return Double.NaN;
    }

    public static double getHoodAngleFromVelocity(double distance, double velocity) {
        return getHoodAngleFromVelocity(distance, velocity, Launcher.preferredHoodAngleLUT.get(distance));
    }

    /**
     * Helper to calculate required velocity.
     * Includes check for negative denominator to prevent NaN.
     */
    public static double calculateVelocity(double x, double deltaY, double angleHoriz, double g) {
        double angleRad = Math.toRadians(angleHoriz);
        double tanTheta = Math.tan(angleRad);
        double cosTheta = Math.cos(angleRad);

        // Denominator represents the vertical distance between the projected
        // angle line and the target. It must be positive.
        double denom = 2 * cosTheta * cosTheta * (x * tanTheta - deltaY);

        if (denom <= 1e-9) return Double.NaN; // Shot aims too low or infinite velocity

        return Math.sqrt((g * x * x) / denom);
    }

    private static boolean isAngleValid(double angleHoriz, double minLipHoriz, double epsilon) {
        double angleVert = 90.0 - angleHoriz;
        return angleHoriz >= (minLipHoriz - epsilon)
                && angleVert >= (MIN_HOOD_ANGLE - epsilon)
                && angleVert <= (MAX_HOOD_ANGLE + epsilon);
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
    @Deprecated
    public static double[] legacyDistanceToLauncherValues(double distance) {
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
        double finalAngleVert = 0; // The angle we will use and return (FROM VERTICAL)
        double finalAngleHoriz = 0; // The angle used in the physics calculation (FROM HORIZONTAL)
        boolean forceOverride = false;

        if (distance <= 1) {
            finalAngleVert = MIN_HOOD_ANGLE;
            finalAngleHoriz = 90.0 - MIN_HOOD_ANGLE; // Convert to horizontal system (90-50=40 deg)
            forceOverride = true;
        } else if (distance >= 4) {
            finalAngleVert = MAX_HOOD_ANGLE;
            finalAngleHoriz = 90.0 - MAX_HOOD_ANGLE;
            forceOverride = true;
        } else {
            // TODO: Find out what edge cases are
        }

        if (!forceOverride) {
            if (optimalAngleVert >= MIN_HOOD_ANGLE && optimalAngleVert <= MAX_HOOD_ANGLE) {
                // Case A: Optimal shot is within vertical angle limits. Use it.
                finalAngleVert = optimalAngleVert;
                finalAngleHoriz = optimalAngleHoriz;

                // Check velocity limit for this optimal shot
                if (minVelocity > LAUNCHER_MAX_BALL_VELOCITY) {
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

        // --- 4. Final Velocity Constraint Check and Return ---\
        if (requiredVelocity > MAX_DRIVE_VELOCITY) {
            // The required velocity for the forced angle is too high. IMPOSSIBLE.
            return new double[]{Double.NaN, Double.NaN};
        }

        // Return the valid, constrained solution
        return new double[]{requiredVelocity, finalAngleVert};
    }

    public static class VirtualGoalSolver {

        public static class ShotSolution {
            public final Rotation2d turretGlobalHeading; // Field-relative aiming angle
            public final double effectiveDistance;       // Distance to feed the shooter map
            public final double turretAngularVelocity;   // Feedforward velocity (rad/s)

            public ShotSolution(Rotation2d heading, double dist, double angVel) {
                this.turretGlobalHeading = heading;
                this.effectiveDistance = dist;
                this.turretAngularVelocity = angVel;
            }
        }

        /**
         * Calculates the turret target heading and velocity feedforward.
         * @param robotPose Current robot field position
         * @param robotVel Current robot field velocity
         * @param robotOmega Current robot angular velocity (rad/s)
         * @param goalPose Goal position
         * @return ShotSolution containing target heading and feedforward
         */
        public static ShotSolution solve(Pose2d robotPose, Vector2d robotVel, double robotOmega, Pose2d goalPose, InterpLUT timeOfFlightLUT) {
            double robotHeadingDegrees = robotPose.getRotation().getDegrees();
            Vector2d turretOffsetField = Constants.TURRET_PHYSICAL_OFFSET.rotateBy(robotHeadingDegrees);

            // 1. Calculate Turret-Base Velocity (Linear + Tangential)
            double tanVx = -robotOmega * turretOffsetField.getY();
            double tanVy = robotOmega * turretOffsetField.getX();

            Vector2d turretBaseVel = new Vector2d(
                    robotVel.getX() + tanVx,
                    robotVel.getY() + tanVy
            );

            Vector2d robotPosVec = new Vector2d(robotPose.getX(), robotPose.getY());
            Vector2d turretPosVec = robotPosVec.plus(turretOffsetField);
            Vector2d goalVec = new Vector2d(goalPose.getX(), goalPose.getY());

            // Start by assuming the target is the real goal
            Vector2d virtualGoal = goalVec;
            double currentDistInches = goalVec.minus(turretPosVec).magnitude();

            // 2. Iterative Convergence Loop
            for (int i = 0; i < 5; i++) {
                double distMeters = currentDistInches * 0.0254;
                double[] shotParams = distanceToLauncherValues(distMeters);

                // --- SAFETY CHECK: IMPOSSIBLE SHOT ---
                // If the lookup table returns NaN (too close/too far), abort prediction.
                if (Double.isNaN(shotParams[0]) || Double.isNaN(shotParams[1])) {
                    // Calculate vector to the REAL goal (no lead)
                    Vector2d turretToRealGoal = goalVec.minus(turretPosVec);

                    // Just face the goal directly
                    Rotation2d fallbackHeading = new Rotation2d(Math.atan2(turretToRealGoal.getY(), turretToRealGoal.getX()));

                    // Return Safe State:
                    // Heading: Face Goal
                    // Distance: Current real distance
                    // FF: -robotOmega (Cancels out robot rotation so turret stays world-locked)
                    return new ShotSolution(fallbackHeading, distMeters, -robotOmega);
                }
                // -------------------------------------

                double shotVelIps = shotParams[0] * 39.3701;
                double timeOfFlight;

                if (USE_INTERPLUT) {
                    timeOfFlight = timeOfFlightLUT.get(distMeters);
                } else {
                    timeOfFlight = calculateTimeOfFlight(currentDistInches, shotVelIps, shotParams[1]);
                }

                // Total drift = Robot Velocity * (Mechanical Delay + Flight Time)
                double totalDriftTime = BALL_TRANSFER_TIME + timeOfFlight;

                // Shift the target opposite to our velocity
                virtualGoal = goalVec.minus(turretBaseVel.scale(totalDriftTime));
                currentDistInches = virtualGoal.minus(turretPosVec).magnitude();
            }

            // 3. Final Calculations (Only runs if shot was valid)
            Vector2d turretToVirtual = virtualGoal.minus(turretPosVec);
            Rotation2d targetHeading = new Rotation2d(Math.atan2(turretToVirtual.getY(), turretToVirtual.getX()));

            // 4. Feedforward Calculation
            double distSq = turretToVirtual.magnitude() * turretToVirtual.magnitude();
            double turretOmegaField = 0.0;

            if (distSq > 0.1) {
                // Cross product for angular velocity tracking
                turretOmegaField = (turretBaseVel.getY() * turretToVirtual.getX() - turretBaseVel.getX() * turretToVirtual.getY()) / distSq;
            }

            // Return relative feedforward (Field Tracking Speed - Robot Rotation Speed)
            return new ShotSolution(targetHeading, currentDistInches * 0.0254, turretOmegaField - robotOmega);
        }

        private static double calculateTimeOfFlight(double distance, double velocity, double angleFromVerticalDeg) {
            double angleRad = Math.toRadians(angleFromVerticalDeg);
            double horizontalVel = velocity * Math.sin(angleRad);

            // Prevent divide by zero
            if (Math.abs(horizontalVel) < 0.01) {
                return 0.0;
            }

            return distance / horizontalVel;
        }
    }

    /**
     * Projects the robot's pose forward in time to account for system latency.
     * * @param currentPose Current Pose from Odometry
     * @param velocity    Current Field-Centric Velocity Vector
     * @param omega       Current Angular Velocity (rad/s)
     * @param latencySec  Latency to compensate for (e.g., 0.05 for 50ms)
     * @return Predicted Pose2d
     */
    public static Pose2d compensateLatency(Pose2d currentPose, Vector2d velocity, double omega, double latencySec) {
        // 1. Project X and Y
        double predictedX = currentPose.getX() + (velocity.getX() * latencySec);
        double predictedY = currentPose.getY() + (velocity.getY() * latencySec);

        // 2. Project Heading (Critical for Turrets!)
        // We get the current heading in radians, add the rotation that will happen,
        // and create a new Rotation2d which handles the wrapping (-PI to PI) automatically.
        double currentHeadingRad = currentPose.getRotation().getRadians();
        double predictedHeadingRad = currentHeadingRad + (omega * latencySec);

        return new Pose2d(predictedX, predictedY, new Rotation2d(predictedHeadingRad));
    }
}
