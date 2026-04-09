package org.firstinspires.ftc.teamcode.gvf;

import com.seattlesolvers.solverslib.geometry.Vector2d;

/**
 * Data container for GVF output at a specific point in time.
 */
public class PathData {
    public final Vector2d vel;
    public final Vector2d accel;
    public final double r;
    public final double power;
    public final double targetHeading;
    public final boolean reversed;
    public final int index;

    /**
     * Constructs PathData with target motion parameters.
     * @param vel target velocity vector
     * @param accel target acceleration vector
     * @param r radius of curvature
     * @param power target power level
     * @param targetHeading target heading in radians
     * @param reversed true if path is reversed
     * @param index current path segment index
     */
    public PathData(Vector2d vel, Vector2d accel, double r, double power, double targetHeading, boolean reversed, int index) {
        this.vel = vel;
        this.accel = accel;
        this.r = r;
        this.power = power;
        this.targetHeading = targetHeading;
        this.reversed = reversed;
        this.index = index;
    }
}
