package org.firstinspires.ftc.teamcode.globals;

import static com.sun.tools.doclint.Entity.pi;
import static org.firstinspires.ftc.teamcode.globals.Constants.GRAVITY;
import static org.firstinspires.ftc.teamcode.globals.Constants.LAUNCHER_HEIGHT;
import static org.firstinspires.ftc.teamcode.globals.Constants.LAUNCHER_MAX_BALL_VELOCITY;
import static org.firstinspires.ftc.teamcode.globals.Constants.MAX_DRIVE_VELOCITY;
import static org.firstinspires.ftc.teamcode.globals.Constants.MAX_HOOD_ANGLE;
import static org.firstinspires.ftc.teamcode.globals.Constants.MIN_HOOD_ANGLE;
import static org.firstinspires.ftc.teamcode.globals.Constants.TARGET_HEIGHT;

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

        if (distance <= 0.825) {
            finalAngleVert = MIN_HOOD_ANGLE;
            finalAngleHoriz = 90.0 - MIN_HOOD_ANGLE; // Convert to horizontal system (90-50=40 deg)
        } else if (distance >= 1.8) {
            finalAngleHoriz = MAX_HOOD_ANGLE;
            finalAngleVert = 90.0 - MAX_HOOD_ANGLE;
        } else {
            // TODO: Find out what edge cases are
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

    /**
     * Calculates the necessary hood angle to hit the target given the current flywheel velocity.
     *
     * @param distance The horizontal distance to the target (x) in meters.
     * @param velocity The current launcher velocity in meters/second.
     * @return distance: The required hood angle (in degrees from vertical).
     * impossible: If the shot is impossible
     * TODO: Fix this method to handle ball vel
     */
    public static Object[] getHoodAngleFromVelocity(double distance, double velocity) {
        double g = GRAVITY;
        double y = TARGET_HEIGHT - LAUNCHER_HEIGHT;
        double x = distance;
        double v2 = Math.pow(velocity, 2);
        double x2 = Math.pow(x, 2);

        // --- 1. Discriminant Check (Physics Impossibility) ---
        // If the target is physically out of range (too far for this speed)
        double a = g * x2;
        double b = -2 * v2 * x;
        double c = (2 * v2 * y) + (g * x2);
        double discriminant = (b * b) - (4 * a * c);

        if (discriminant < 0) {
            // We cannot reach the target.
            // Best effort: Return MAX_HOOD_ANGLE for maximum range.
            return new Object[]{MAX_HOOD_ANGLE, true};
        }

        // --- 2. Calculate Ideal Angles ---
        double tanTheta1 = (-b - Math.sqrt(discriminant)) / (2 * a);
        double tanTheta2 = (-b + Math.sqrt(discriminant)) / (2 * a);

        // Convert to Vertical System
        double angleVert1 = 90.0 - Math.toDegrees(Math.atan(tanTheta1));
        double angleVert2 = 90.0 - Math.toDegrees(Math.atan(tanTheta2));

        // --- 3. Pick the Best "Theoretical" Angle ---
        // We prefer the "flatter" shot (larger vertical angle) if available
        // because it spends less time in the air.
        double bestAngle = Math.max(angleVert1, angleVert2);

        // --- 4. Validation & Clamping (Hardware Limits) ---
        if (bestAngle >= MIN_HOOD_ANGLE && bestAngle <= MAX_HOOD_ANGLE) {
            // Perfect scenario: We can hit it exactly.
            return new Object[]{bestAngle, false};
        } else {
            if (bestAngle < MIN_HOOD_ANGLE) {
                // Needed a steeper shot -> limit to MIN
                return new Object[]{MIN_HOOD_ANGLE, true};
            } else {
                // Needed a flatter shot -> limit to MAX
                return new Object[]{MAX_HOOD_ANGLE, true};
            }
        }
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

        ;

        public ShootingMath(Position target, double ballRadius, double robotHeight) {
            this.target = target;
            this.ballRadius = ballRadius;
            this.robotHeight = robotHeight;
        }


        /**
         * predicts the hood angle, turret angle, and flywheel speed of the robot when aiming while moving
         * uses the robotPose, and robotSpeed.
         */
        public ShootingMath.PredictResult predict(Pose2d robotPose, ChassisSpeeds robotSpeed) {
            //make a new PredictResult
            ShootingMath.PredictResult result = new PredictResult();
            //variables adjusted with robot height and ball radius
            final double targetX = target.x - Math.signum(target.x) * ballRadius;
            final double targetY = target.y - ballRadius;
            final double targetZ = target.z - robotHeight - ballRadius;
            final double robotX = robotPose.getX();
            final double robotY = robotPose.getY();
            final double inchesPerMeters = 39.3701;

            RobotLog.aa("OriginalTargetPosition", String.valueOf(target));

            RobotLog.aa("AdjustedTargetPosition", String.format("(%.03f, %.03f, %.03f)", targetX, targetY, targetZ));

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

            RobotLog.aa("ShootingState", String.format("distance:%.03f, time:%.03f, vh:%.03f, vz:%.03f, alpha:%.03f", distance, time, vh, vz, Math.toDegrees(alpha)));

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

            RobotLog.aa("ShootingState2", String.format("vx:%.03f, vy:%.03f, vfx:%03f, vfy:%.03f, vfh:%.03f, vf:%.03f", vx, vy, vfx, vfy, vfh, vf));

            //returning turret angle, hood angle, and fly wheel speed
            result.turretAngle = Math.atan2(vfy, vfx) - robotPose.getHeading();
            result.hoodAngle = Math.atan2(vz, vfh);
            result.flyWheelSpeed = vf / inchesPerMeters;//meters per second
            result.success = true;

            return result;
        }
    }


}
