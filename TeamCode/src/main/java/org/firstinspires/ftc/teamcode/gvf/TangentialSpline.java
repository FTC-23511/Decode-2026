package org.firstinspires.ftc.teamcode.gvf;

import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;

/**
 * Implementation for curved paths where heading is tangential to movement.
 * Uses Quintic Hermite Splines for G2 continuity (smooth position, velocity, and acceleration).
 */
public class TangentialSpline extends org.firstinspires.ftc.teamcode.gvf.Spline {
    // Polynomial coefficients: f(t) = c5*t^5 + c4*t^4 + c3*t^3 + c2*t^2 + c1*t + c0
    private final Vector2d c5, c4, c3, c2, c1, c0;
    private final double length;

    // Gaussian Quadrature constants for arc length approximation
    private static final double[] GL_T = {0.0, 0.5384693101056831, -0.5384693101056831, 0.9061798459386640, -0.9061798459386640};
    private static final double[] GL_W = {0.5688888888888889, 0.4786286704993665, 0.4786286704993665, 0.2369268850561891, 0.2369268850561891};

    /**
     * Constructs a tangential spline between two poses.
     * @param start start pose
     * @param end end pose
     */
    public TangentialSpline(Pose2d start, Pose2d end) {
        this(start, end, start.getTranslation().getDistance(end.getTranslation()) * 1.2,
                start.getTranslation().getDistance(end.getTranslation()) * 1.2);
    }

    /**
     * Constructs a tangential spline between two poses with explicit tangent magnitudes.
     * @param start start pose
     * @param end end pose
     * @param startMag magnitude of the tangent vector at the start
     * @param endMag magnitude of the tangent vector at the end
     */
    public TangentialSpline(Pose2d start, Pose2d end, double startMag, double endMag) {
        Vector2d p0 = new Vector2d(start.getX(), start.getY());
        Vector2d p1 = new Vector2d(end.getX(), end.getY());

        Vector2d v0 = new Vector2d(Math.cos(start.getHeading()), Math.sin(start.getHeading())).times(startMag);
        Vector2d v1 = new Vector2d(Math.cos(end.getHeading()), Math.sin(end.getHeading())).times(endMag);

        // Quintic Hermite Spline coefficients for zero acceleration at endpoints
        c5 = p0.times(-6.0).minus(v0.times(3.0)).plus(p1.times(6.0)).minus(v1.times(3.0));
        c4 = p0.times(15.0).plus(v0.times(8.0)).minus(p1.times(15.0)).plus(v1.times(7.0));
        c3 = p0.times(-10.0).minus(v0.times(6.0)).plus(p1.times(10.0)).minus(v1.times(4.0));
        c2 = new Vector2d(0, 0);
        c1 = v0;
        c0 = p0;

        this.length = computeArcLength(1.0);
    }

    /**
     * Returns the position vector at the given parameter t [0, 1].
     */
    @Override
    public Vector2d getPos(double t) {
        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double t5 = t4 * t;
        return c5.times(t5)
                .plus(c4.times(t4))
                .plus(c3.times(t3))
                .plus(c2.times(t2))
                .plus(c1.times(t))
                .plus(c0);
    }

    /**
     * Returns the velocity vector at the given parameter t [0, 1].
     */
    @Override
    public Vector2d getVel(double t) {
        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        return c5.times(5.0 * t4)
                .plus(c4.times(4.0 * t3))
                .plus(c3.times(3.0 * t2))
                .plus(c2.times(2.0 * t))
                .plus(c1);
    }

    /**
     * Returns the acceleration vector at the given parameter t [0, 1].
     */
    @Override
    public Vector2d getAccel(double t) {
        double t2 = t * t;
        double t3 = t2 * t;
        return c5.times(20.0 * t3)
                .plus(c4.times(12.0 * t2))
                .plus(c3.times(6.0 * t))
                .plus(c2.times(2.0));
    }

    /**
     * Computes the arc length from 0 to t using Gaussian Quadrature.
     */
    private double computeArcLength(double t) {
        if (t <= 0.0) return 0.0;
        double len = 0.0;
        double halfT = t / 2.0;
        for (int i = 0; i < 5; i++) {
            double tau = halfT * GL_T[i] + halfT;
            len += GL_W[i] * getVel(tau).magnitude();
        }
        return len * halfT;
    }

    @Override
    public double getLength() {
        return length;
    }

    @Override
    public double getR(double t) {
        Vector2d v = getVel(t);
        Vector2d a = getAccel(t);
        double vx = v.getX();
        double vy = v.getY();
        double ax = a.getX();
        double ay = a.getY();

        double speedSq = vx * vx + vy * vy;
        double denom = speedSq * Math.sqrt(speedSq);
        if (denom < 1e-6) return MAX_RADIUS;

        double kappa = (vx * ay - vy * ax) / denom;
        if (Math.abs(kappa) < 1e-9) return MAX_RADIUS;

        double radius = 1.0 / kappa;
        return Math.abs(radius) > MAX_RADIUS ? Math.signum(radius) * MAX_RADIUS : radius;
    }

    @Override
    public double getT(Pose2d pose) {
        Vector2d target = new Vector2d(pose.getX(), pose.getY());
        double bestT = 0.0;
        double minDistSq = Double.MAX_VALUE;

        // Initial coarse search
        for (int i = 0; i <= 20; i++) {
            double t_sample = i / 20.0;
            Vector2d error = getPos(t_sample).minus(target);
            double distSq = error.dot(error);
            if (distSq < minDistSq) {
                minDistSq = distSq;
                bestT = t_sample;
            }
        }

        // Refine using Newton's method
        double t_opt = bestT;
        for (int i = 0; i < 10; i++) {
            Vector2d p = getPos(t_opt);
            Vector2d v = getVel(t_opt);
            Vector2d acc = getAccel(t_opt);
            Vector2d error = p.minus(target);

            double f_val = error.dot(v);
            double df_val = v.dot(v) + error.dot(acc);

            if (Math.abs(df_val) < 1e-6) break;
            double step = f_val / df_val;
            t_opt = Math.max(0.0, Math.min(1.0, t_opt - step));
            if (Math.abs(step) < 1e-6) break;
        }
        return t_opt;
    }

    @Override
    public double getTargetHeading(double t) {
        Vector2d v = getVel(t);
        return Math.atan2(v.getY(), v.getX());
    }
}
