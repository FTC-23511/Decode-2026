package org.firstinspires.ftc.teamcode.gvf;

/**
 * Representation of a single segment in a GVF Path.
 */
public class PathSegment {
    public final Spline spline;
    public final boolean reversed;
    public boolean decel;
    public final double power;

    /**
     * Constructs a path segment.
     * @param s spline implementation
     * @param r true if reversed
     * @param d true if deceleration is enabled
     * @param p segment-specific power scalar
     */
    public PathSegment(Spline s, boolean r, boolean d, double p) {
        this.spline = s;
        this.reversed = r;
        this.decel = d;
        this.power = p;
    }
}
