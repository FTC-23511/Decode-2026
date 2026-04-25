package com.seattlesolvers.solverslib.geometry;

/**
 * A wrapper for {@link Pose2d} for configuration on FTC Dashboard
 */
public class DashPose2d {
    public double x;
    public double y;
    public double rotation;

    /**
     * Constructs a pose at the origin facing toward the positive X axis.
     * (Translation2d{0, 0} and Rotation{0})
     */
    public DashPose2d(double x, double y , double rotation) {
        this.x = x;
        this.y = y;
        this.rotation = rotation;
    }

    public Pose2d get() {
        return new Pose2d(x, y, Math.toRadians(rotation));
    }
}