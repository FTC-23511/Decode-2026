package org.firstinspires.ftc.teamcode.globals;

import static com.sun.tools.doclint.Entity.pi;
import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
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

    /**
     * Calculates the flattest possible launch parameters (highest velocity, lowest angle)
     * by aiming for the backboard rather than the goal center.
     * * <p>This method optimizes for a "line-drive" shot by targeting a vertical offset
     * (BACKBOARD_Y_OFFSET) above the goal height. It ensures the trajectory clears
     * the goal lip (GOAL_LIP) at the front of the goal by enforcing a minimum geometric
     * launch angle before solving for velocity.</p>
     *
     * @param distance The horizontal distance from the launcher to the center of the goal (meters).
     * @return A 2-item double array containing:
     * <ul>
     * <li>[0]: Required launch velocity (m/s).</li>
     * <li>[1]: Required hood angle measured FROM THE VERTICAL (degrees).</li>
     * </ul>
     * Returns {@code [Double.NaN, Double.NaN]} if the target is out of range,
     * mechanically unreachable, or cannot clear the goal lip.
     */
    public static double[] distanceToLauncherValues(double distance) {
        double g = GRAVITY;
        double x = distance;

        // Target the backboard vertically to maintain higher velocity and flatter arc
        double targetY = TARGET_HEIGHT + BACKBOARD_Y_OFFSET;
        double deltaYBackboard = targetY - LAUNCHER_HEIGHT;
        double deltaYLip = TARGET_HEIGHT - LAUNCHER_HEIGHT;

        // --- 1. Define Mechanical Limits (Vertical to Horizontal conversion) ---
        double minPhysAngleHoriz = 90.0 - MAX_HOOD_ANGLE;
        double maxPhysAngleHoriz = 90.0 - MIN_HOOD_ANGLE;

        // --- 2. Define Geometric Lip Constraint ---
        double xLip = distance - GOAL_LIP;
        double minLipAngleHoriz = 0.0;

        if (xLip > 0) {
            // Calculate the minimum angle required to be at TARGET_HEIGHT when at xLip
            // while the endpoint is deltaYBackboard at distance x.
            double numerator = (deltaYBackboard * Math.pow(xLip, 2)) - (deltaYLip * Math.pow(x, 2));
            double denominator = (x * Math.pow(xLip, 2)) - (xLip * Math.pow(x, 2));

            // Add x degree safety buffer to ensure we don't graze the rim
            minLipAngleHoriz = Math.toDegrees(Math.atan(numerator / denominator)) + LIP_BUFFER;
        }

        // --- 3. Determine Final Target Angle ---
        // Start with the flattest possible angle (hood limit) and steepen only if lip requires it
        double targetAngleHoriz = Math.max(minPhysAngleHoriz, minLipAngleHoriz);

        // Impossible if the lip requires an angle steeper than our hood can physically provide
        if (targetAngleHoriz > maxPhysAngleHoriz) {
            return new double[]{Double.NaN, Double.NaN};
        }

        // --- 4. Calculate Velocity Required for the Backboard Target ---
        double vRequired = calculateVelocity(x, deltaYBackboard, targetAngleHoriz, g);

        double finalVelocity;
        double finalAngleHoriz;

        // --- 5. Velocity Constraint Handling ---
        if (!Double.isNaN(vRequired) && vRequired <= LAUNCHER_MAX_BALL_VELOCITY) {
            finalVelocity = vRequired;
            finalAngleHoriz = targetAngleHoriz;
        } else {
            // If the flattest shot is too fast, cap at MAX velocity and solve for the angle
            finalVelocity = LAUNCHER_MAX_BALL_VELOCITY;

            double v = LAUNCHER_MAX_BALL_VELOCITY;
            double A = (g * x * x) / (2 * v * v);
            double B = -x;
            double C = deltaYBackboard + A;

            double discriminant = B * B - 4 * A * C;

            if (discriminant < 0) {
                return new double[]{Double.NaN, Double.NaN};
            }

            // Solve quadratic for the two possible angles (flatter root and steeper root)
            double tanTheta1 = (-B - Math.sqrt(discriminant)) / (2 * A);
            double tanTheta2 = (-B + Math.sqrt(discriminant)) / (2 * A);

            double angle1 = Math.toDegrees(Math.atan(tanTheta1));
            double angle2 = Math.toDegrees(Math.atan(tanTheta2));

            // Prioritize the flatter solution that meets all safety constraints
            if (angle1 >= targetAngleHoriz && angle1 <= maxPhysAngleHoriz) {
                finalAngleHoriz = angle1;
            } else if (angle2 >= targetAngleHoriz && angle2 <= maxPhysAngleHoriz) {
                finalAngleHoriz = angle2;
            } else {
                return new double[]{Double.NaN, Double.NaN};
            }
        }

        return new double[]{finalVelocity, 90.0 - finalAngleHoriz};
    }

    /**
     * Helper to calculate required velocity for a specific angle
     */
    private static double calculateVelocity(double x, double deltaY, double angleHoriz, double g) {
        double angleRad = Math.toRadians(angleHoriz);
        double tanTheta = Math.tan(angleRad);
        double cosTheta = Math.cos(angleRad);

        // v = sqrt( (g*x^2) / (2*cos^2(theta) * (x*tan(theta) - y)) )
        double denom = 2 * cosTheta * cosTheta * (x * tanTheta - deltaY);

        if (denom <= 0) return Double.NaN;
        return Math.sqrt((g * x * x) / denom);
    }

    /**
     * Calculates the flattest possible hood angle for a specific velocity and distance.
     * Prioritizes hitting the backboard, but falls back to the goal center if the
     * current velocity is insufficient to reach the higher target.
     *
     * @param distance The horizontal distance to the target (m).
     * @param velocity The current ball launch velocity (m/s).
     * @return The angle from VERTICAL (degrees). Returns NaN if the velocity is
     * physically insufficient to reach even the minimum goal height.
     */
    public static double getHoodAngleFromVelocity(double distance, double velocity) {
        double g = GRAVITY;
        double x = distance;
        double xLip = x - GOAL_LIP;
        double deltaYLip = TARGET_HEIGHT - LAUNCHER_HEIGHT;

        // 1. Define the two possible target heights
        double yBackboard = TARGET_HEIGHT + BACKBOARD_Y_OFFSET;
        double yCenter = TARGET_HEIGHT;

        // 2. Try to solve for the Backboard first
        double angle = solveForTarget(x, yBackboard, xLip, deltaYLip, velocity, g);

        // 3. Fallback: If backboard is impossible, try the Goal Center
        if (Double.isNaN(angle)) {
            angle = solveForTarget(x, yCenter, xLip, deltaYLip, velocity, g);
        }

        return angle; // Returns angle from vertical, or NaN if both failed
    }

    /**
     * Internal helper to calculate the angle for a specific Y target.
     */
    private static double solveForTarget(double x, double targetY, double xLip, double deltaYLip, double v, double g) {
        double deltaY = targetY - LAUNCHER_HEIGHT;

        // Quadratic terms for trajectory: A*tan^2(theta) + B*tan(theta) + C = 0
        double A = (g * x * x) / (2.0 * v * v);
        double B = -x;
        double C = deltaY + A;

        double discriminant = (B * B) - (4.0 * A * C);
        if (discriminant < 0) return Double.NaN; // Velocity too low for this height

        double sqrtD = Math.sqrt(discriminant);
        double tanTheta1 = (-B - sqrtD) / (2.0 * A); // Flatter
        double tanTheta2 = (-B + sqrtD) / (2.0 * A); // Steeper

        double angle1H = Math.toDegrees(Math.atan(tanTheta1));
        double angle2H = Math.toDegrees(Math.atan(tanTheta2));

        // Calculate minimum angle to clear lip while hitting THIS specific deltaY
        double minLipH = 0;
        if (xLip > 0) {
            // Geometric clearance: (targetY at x) must not be blocked by (TARGET_HEIGHT at xLip)
            double num = (deltaY * xLip * xLip) - (deltaYLip * x * x);
            double den = (x * xLip * xLip) - (xLip * x * x);
            minLipH = Math.toDegrees(Math.atan(num / den)) + LIP_BUFFER;
        }

        // Selection Logic (Priority: Flattest valid solution)
        double epsilon = 1e-7;
        if (isAngleValid(angle1H, minLipH, epsilon)) return 90.0 - angle1H;
//        if (isAngleValid(angle2H, minLipH, epsilon)) return 90.0 - angle2H;

        return Double.NaN;
    }

    private static boolean isAngleValid(double angleHoriz, double minLipHoriz, double epsilon) {
        double angleVert = 90.0 - angleHoriz;
        return angleHoriz >= (minLipHoriz - epsilon)
                && angleVert >= (MIN_HOOD_ANGLE - epsilon)
                && angleVert <= (MAX_HOOD_ANGLE + epsilon);
    }

    /**
     * Finds leg of triangle given hypotenuse and the other leg
     */
    public static double findLeg(double hypot, double leg) {
        if (leg > hypot || hypot <= 0 || leg <= 0) {
            return Double.NaN;
        }

        return Math.sqrt(Math.pow(hypot, 2) - Math.pow(leg, 2));
    }

    /**
     * Finds hypotenuse of triangle given both legs
     */
    public static double findHypotenuse(double leg1, double leg2) {
        return Math.hypot(leg1, leg2);
    }

    /**
     * Sub-class for shooting while moving math
     */
    public static class ShootingMath {
        Position target;
        double ballRadius;
        Pose2D robot;
        ChassisSpeeds speed;
        double robotHeight;

        public static class PredictResult {
            public boolean success = false;
            public double flyWheelSpeed = 0; // m/s
            public double turretAngle = 0; // radian
            public double hoodAngle = 0; // radian

            @Override
            public String toString() {
                return String.format("PredictResults(success=%s, flywheelSpeed=%.03f m/s, turretAngle=%.03f degrees, hoodAngle=%.03f degrees)",
                        success ? "true" : "false", flyWheelSpeed, turretAngle * 180.0 / Math.PI, hoodAngle * 180.0 / Math.PI);
            }
        }

        public ShootingMath(Position target, double ballRadius, double robotHeight) {
            this.target = target;
            this.ballRadius = ballRadius;
            this.robotHeight = robotHeight;
        }


        /**
         * predicts the hood angle, turret angle, and flywheel speed of the robot when aiming while moving
         * uses the robotPose, and robotSpeed.
         */
        public PredictResult predict(Pose2d robotPose, ChassisSpeeds robotSpeed) {
            //make a new PredictResult
            PredictResult result = new PredictResult();
            //variables adjusted with robot height and ball radius
            final double targetX = target.x - Math.signum(target.x) * ballRadius;
            final double targetY = target.y - ballRadius;
            final double targetZ = target.z - robotHeight - ballRadius;
            final double robotX = robotPose.getX();
            final double robotY = robotPose.getY();
            final double inchesPerMeters = 39.3701;
//
//            RobotLog.aa("OriginalTargetPosition", String.valueOf(target));

//            RobotLog.aa("AdjustedTargetPosition", String.format("(%.03f, %.03f, %.03f)", targetX, targetY, targetZ));

            //mathhhhhh
            final double gravity = GRAVITY * inchesPerMeters;
            //finds the horizontal distance between the robot and the target
            final double distance = Math.sqrt(Math.pow(targetX - robotX, 2) + Math.pow(targetY - robotY, 2));
            //ball's travel time
            final double time = Math.sqrt(2 * ((targetZ) + distance) / gravity);
            //horizontal speed of the ball
            final double vh = distance / time;
            //vertical speed of the ball
            final double vz = gravity * (time) - vh;
            //angle of shooting
            final double alpha = Math.atan2((targetY - robotY), (targetX - robotX));

//            RobotLog.aa("ShootingState", String.format("distance:%.03f, time:%.03f, vh:%.03f, vz:%.03f, alpha:%.03f", distance, time, vh, vz, Math.toDegrees(alpha)));

            //x component of the speed of the ball
            final double vx = Math.cos(alpha) * vh;
            //y component of the speed of the ball
            final double vy = Math.sin(alpha) * vh;
            //x component of the speed of the flywheel
            final double vfx = (vx) - robotSpeed.vxMetersPerSecond * inchesPerMeters;
            //y component of the speed of the flywheel
            final double vfy = (vy) - robotSpeed.vyMetersPerSecond * inchesPerMeters;
            //horizontal component of the speed of the flywheel
            final double vfh = Math.sqrt(Math.pow(vfx, 2) + Math.pow(vfy, 2));
            //flywheel speed
            final double vf = Math.sqrt(Math.pow(vfh, 2) + Math.pow(vz, 2));

//            RobotLog.aa("ShootingState2", String.format("vx:%.03f, vy:%.03f, vfx:%03f, vfy:%.03f, vfh:%.03f, vf:%.03f", vx, vy, vfx, vfy, vfh, vf));

            //returning turret angle, hood angle, and fly wheel speed
            result.turretAngle = Math.atan2(vfy, vfx) - robotPose.getHeading();
            result.hoodAngle = Math.atan2(vz, vfh);
            result.flyWheelSpeed = vf / inchesPerMeters;//meters per second
            result.success = true;

            return result;
        }
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
}
