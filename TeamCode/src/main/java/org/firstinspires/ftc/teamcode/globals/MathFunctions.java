package org.firstinspires.ftc.teamcode.globals;

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
     *
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
}
