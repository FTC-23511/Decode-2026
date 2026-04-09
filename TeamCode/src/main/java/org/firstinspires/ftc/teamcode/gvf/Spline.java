package org.firstinspires.ftc.teamcode.gvf;

import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;

/**
 * Base class for all path elements.
 * Provides kinematic and orientation data at any point t [0, 1].
 */
public class Spline {
    public static final double MAX_RADIUS = 144.0;

    /**
     * Returns the position vector at the given parameter t [0, 1].
     * @param t normalized path parameter
     * @return position as a Vector2d
     */
    public Vector2d getPos(double t) {
        return new Vector2d(0, 0);
    }

    /**
     * Returns the velocity vector at the given parameter t [0, 1].
     * @param t normalized path parameter
     * @return velocity as a Vector2d
     */
    public Vector2d getVel(double t) {
        return new Vector2d(0, 0);
    }

    /**
     * Returns the acceleration vector at the given parameter t [0, 1].
     * @param t normalized path parameter
     * @return acceleration as a Vector2d
     */
    public Vector2d getAccel(double t) {
        return new Vector2d(0, 0);
    }

    /**
     * Returns the total arc length of the spline.
     * @return length in inches
     */
    public double getLength() {
        return 0;
    }

    /**
     * Returns the radius of curvature at parameter t [0, 1].
     * @param t normalized path parameter
     * @return radius in inches
     */
    public double getR(double t) {
        return MAX_RADIUS;
    }

    /**
     * Computes the closest parameter t for a given robot pose.
     * @param pose current robot pose
     * @return normalized parameter t [0, 1]
     */
    public double getT(Pose2d pose) {
        return 0;
    }

    /**
     * Returns the target heading at parameter t [0, 1].
     * @param t normalized path parameter
     * @return target heading in radians
     */
    public double getTargetHeading(double t) {
        return 0;
    }
}
