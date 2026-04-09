package org.firstinspires.ftc.teamcode.gvf;

/**
 * An exponential repulsive obstacle node used by the Guiding Vector Field.
 */
public class RepulsionPoint {
    private final double x;
    private final double y;
    private final double weight;

    /**
     * Constructs a repulsion point.
     * @param x x-coordinate in inches
     * @param y y-coordinate in inches
     * @param weight influence weight
     */
    public RepulsionPoint(double x, double y, double weight) {
        this.x = x;
        this.y = y;
        this.weight = weight;
    }

    /**
     * Returns the x-coordinate of the point.
     * @return x in inches
     */
    public double getX() {
        return x;
    }

    /**
     * Returns the y-coordinate of the point.
     * @return y in inches
     */
    public double getY() {
        return y;
    }

    /**
     * Returns the influence weight of the point.
     * @return weight scalar
     */
    public double getWeight() {
        return weight;
    }
}
