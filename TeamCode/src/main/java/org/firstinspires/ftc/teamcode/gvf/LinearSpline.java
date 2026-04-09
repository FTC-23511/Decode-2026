package org.firstinspires.ftc.teamcode.gvf;

import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.util.MathUtils;

/**
 * Implementation for straight line segments where heading is linearly interpolated.
 */
public class LinearSpline extends Spline {
    private final double x0;
    private final double y0;
    private final double x1;
    private final double y1;
    private final double h0;
    private final double h1;
    private final double length;

    /**
     * Constructs a linear spline between two poses.
     * @param start start pose
     * @param end end pose
     */
    public LinearSpline(Pose2d start, Pose2d end) {
        this.x0 = start.getX();
        this.y0 = start.getY();
        this.x1 = end.getX();
        this.y1 = end.getY();
        this.h0 = start.getHeading();
        this.h1 = end.getHeading();

        double dx = x1 - x0;
        double dy = y1 - y0;
        this.length = Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Returns the position vector at the given parameter t [0, 1].
     * @param t normalized path parameter
     * @return position as a Vector2d
     */
    @Override
    public Vector2d getPos(double t) {
        double px = x0 + t * (x1 - x0);
        double py = y0 + t * (y1 - y0);
        return new Vector2d(px, py);
    }

    /**
     * Returns the velocity vector at the given parameter t [0, 1].
     * @param t normalized path parameter
     * @return velocity as a Vector2d
     */
    @Override
    public Vector2d getVel(double t) {
        double vx = x1 - x0;
        double vy = y1 - y0;
        return new Vector2d(vx, vy);
    }

    /**
     * Returns the acceleration vector at the given parameter t [0, 1].
     * @param t normalized path parameter
     * @return acceleration as a Vector2d (zero for linear)
     */
    @Override
    public Vector2d getAccel(double t) {
        return new Vector2d(0.0, 0.0);
    }

    /**
     * Returns the total arc length of the spline.
     * @return length in inches
     */
    @Override
    public double getLength() {
        return length;
    }

    /**
     * Returns the radius of curvature at parameter t [0, 1].
     * @param t normalized path parameter
     * @return radius in inches (returns MAX_RADIUS for linear)
     */
    @Override
    public double getR(double t) {
        return MAX_RADIUS;
    }

    /**
     * Computes the closest parameter t for a given robot pose using projection.
     * @param pose current robot pose
     * @return normalized parameter t [0, 1]
     */
    @Override
    public double getT(Pose2d pose) {
        if (length < 1e-6) {
            return 0.0;
        }
        double dx = x1 - x0;
        double dy = y1 - y0;
        double px = pose.getX() - x0;
        double py = pose.getY() - y0;
        double t_val = (px * dx + py * dy) / (length * length);
        if (t_val < 0.0) {
            return 0.0;
        }
        if (t_val > 1.0) {
            return 1.0;
        }
        return t_val;
    }

    /**
     * Returns the target heading at parameter t [0, 1] via linear interpolation.
     * @param t normalized path parameter
     * @return target heading in radians
     */
    @Override
    public double getTargetHeading(double t) {
        double diff = MathUtils.normalizeRadians(h1 - h0, false);
        double heading = h0 + t * diff;
        return MathUtils.normalizeRadians(heading, false);
    }
}
