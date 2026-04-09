package org.firstinspires.ftc.teamcode.gvf;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import java.util.ArrayList;
import java.util.List;

/**
 * Guided Vector Field (GVF) Path representation.
 * Computes vector fields to guide the robot along smooth Tangential Splines or Linear paths.
 */
@Config
public class Path {
    public final List<org.firstinspires.ftc.teamcode.gvf.PathSegment> pathSegments = new ArrayList<>();
    public final List<org.firstinspires.ftc.teamcode.gvf.RepulsionPoint> repel = new ArrayList<>();

    private boolean reversed = false;
    private boolean decel = false;
    private boolean completed = false;
    private double power = 1.0;
    private int lastReachedIndex = 0;
    private Pose2d lastPose;
    private double lastTangentMagnitude = 1.0;

    // Tuning constants for GVF
    private static final double K_P = 0.2;
    private static final double K_REPEL = 2.0;
    private static final double REPULSION_DECAY = 4.0;

    /**
     * Constructs a new path starting from a given pose.
     * @param startPose initial robot pose
     */
    public Path(Pose2d startPose) {
        if (startPose == null) {
            this.lastPose = new Pose2d(0, 0, 0);
        } else {
            this.lastPose = new Pose2d(startPose.getX(), startPose.getY(), startPose.getHeading());
        }
        this.lastTangentMagnitude = 1.0;
    }

    /**
     * Adds a tangential spline segment to the path.
     * @param p end pose of the segment
     * @return this path for chaining
     */
    public Path addTangentialPoint(Pose2d p) {
        return addTangentialPoint(p, 1.0);
    }

    /**
     * Adds a tangential spline segment to the path with explicit tangent magnitude control.
     * @param p end pose of the segment
     * @param tangentMagnitude multiplier for the tangent vector magnitude at the waypoint
     * @return this path for chaining
     */
    public Path addTangentialPoint(Pose2d p, double tangentMagnitude) {
        if (p == null) return this;
        Pose2d target = p;
        if (reversed) {
            target = new Pose2d(p.getX(), p.getY(), p.getHeading() + Math.PI);
        }

        double dist = target.getTranslation().getDistance(lastPose.getTranslation());
        double currentScalar = tangentMagnitude * dist * 1.2;

        // If it's the first segment, initialize lastTangentMagnitude based on initial dist
        if (pathSegments.isEmpty()) {
            lastTangentMagnitude = 1.0 * dist * 1.2;
        }

        org.firstinspires.ftc.teamcode.gvf.TangentialSpline s = new org.firstinspires.ftc.teamcode.gvf.TangentialSpline(lastPose, target, lastTangentMagnitude, currentScalar);
        pathSegments.add(new org.firstinspires.ftc.teamcode.gvf.PathSegment(s, reversed, decel, 1.0));

        lastPose = target;
        lastTangentMagnitude = currentScalar;
        return this;
    }

    /**
     * Adds a holonomic spline segment to the path.
     * @param p end pose of the segment
     * @return this path for chaining
     */
    public Path addHolonomicPoint(Pose2d p) {
        return addHolonomicPoint(p, 1.0);
    }

    /**
     * Adds a holonomic spline segment to the path with explicit tangent magnitude control.
     * @param p end pose of the segment
     * @param tangentMagnitude multiplier for the tangent vector magnitude at the waypoint
     * @return this path for chaining
     */
    public Path addHolonomicPoint(Pose2d p, double tangentMagnitude) {
        if (p == null) return this;
        Pose2d target = p;
        if (reversed) {
            target = new Pose2d(p.getX(), p.getY(), p.getHeading() + Math.PI);
        }

        double dist = target.getTranslation().getDistance(lastPose.getTranslation());
        double currentScalar = tangentMagnitude * dist * 1.2;

        // If it's the first segment, initialize lastTangentMagnitude based on initial dist
        if (pathSegments.isEmpty()) {
            lastTangentMagnitude = 1.0 * dist * 1.2;
        }

        org.firstinspires.ftc.teamcode.gvf.HolonomicSpline s = new org.firstinspires.ftc.teamcode.gvf.HolonomicSpline(lastPose, target, lastTangentMagnitude, currentScalar);
        pathSegments.add(new PathSegment(s, reversed, decel, 1.0));

        lastPose = target;
        lastTangentMagnitude = currentScalar;
        return this;
    }

    /**
     * Alias for addTangentialPoint.
     * @param p end pose of the segment
     * @return this path for chaining
     */
    public Path addPoint(Pose2d p) {
        return addTangentialPoint(p);
    }

    /**
     * Adds a linear spline segment to the path.
     * @param p end pose of the segment
     * @return this path for chaining
     */
    public Path addLinearPoint(Pose2d p) {
        if (p == null) return this;
        Pose2d target = p;
        if (reversed) {
            target = new Pose2d(p.getX(), p.getY(), p.getHeading() + Math.PI);
        }
        org.firstinspires.ftc.teamcode.gvf.LinearSpline s = new org.firstinspires.ftc.teamcode.gvf.LinearSpline(lastPose, target);
        pathSegments.add(new PathSegment(s, reversed, decel, 1.0));
        lastPose = target;
        return this;
    }

    /**
     * Adds a repulsion point to influence the vector field.
     * @param point repulsion point to add
     * @return this path for chaining
     */
    public Path addRepel(RepulsionPoint point) {
        if (point != null) {
            this.repel.add(point);
        }
        return this;
    }

    /**
     * Sets whether subsequent segments should be marked as reversed.
     * @param rev true if reversed
     * @return this path for chaining
     */
    public Path setReversed(boolean rev) {
        if (rev != reversed) {
            lastPose = new Pose2d(lastPose.getX(), lastPose.getY(), lastPose.getHeading() + Math.PI);
        }
        this.reversed = rev;
        return this;
    }

    /**
     * Sets whether subsequent segments should have deceleration enabled.
     * @param dec true if deceleration enabled
     * @return this path for chaining
     */
    public Path setDecel(boolean dec) {
        if (dec != decel && !pathSegments.isEmpty()) {
            pathSegments.get(pathSegments.size() - 1).decel = dec;
        }
        this.decel = dec;
        return this;
    }

    /**
     * Sets the global power scalar for the path.
     * @param p power scalar
     * @return this path for chaining
     */
    public Path setPower(double p) {
        this.power = p;
        return this;
    }

    /**
     * Returns whether the path has been fully traversed.
     * @return true if completed
     */
    public boolean isCompleted() {
        return completed;
    }

    /**
     * Returns the last pose in the path.
     * @return last pose
     */
    public Pose2d getLastPose() {
        return lastPose;
    }

    /**
     * Updates the vector field calculation based on the robot's current pose.
     * @param robot current robot pose
     * @return PathData containing target velocity, heading, etc.
     */
    public PathData update(Pose2d robot) {
        if (robot == null || pathSegments.isEmpty()) {
            completed = true;
            return null;
        }

        Vector2d robotPos = new Vector2d(robot.getX(), robot.getY());
        int index = lastReachedIndex;
        double t = pathSegments.get(index).spline.getT(robot);

        if (t >= 0.99 && index < pathSegments.size() - 1) {
            index++;
            lastReachedIndex = index;
            t = pathSegments.get(index).spline.getT(robot);
        }

        if (index == pathSegments.size() - 1 && t >= 0.99) {
            completed = true;
            return null;
        }

        PathSegment curr = pathSegments.get(index);
        Spline spline = curr.spline;

        Vector2d posOnSpline = spline.getPos(t);
        Vector2d tangent = spline.getVel(t);

        double heading = spline.getTargetHeading(t);
        if (curr.reversed && spline instanceof TangentialSpline) {
            heading += Math.PI;
        }

        Vector2d v_t = tangent.normalize();
        Vector2d error = posOnSpline.minus(robotPos);
        Vector2d v_p = error.times(K_P);

        Vector2d v_rep = new Vector2d(0.0, 0.0);
        for (RepulsionPoint rp : repel) {
            Vector2d diff = robotPos.minus(new Vector2d(rp.getX(), rp.getY()));
            double dist = diff.magnitude();
            if (dist > 0.01) {
                double scale = K_REPEL * Math.exp(-dist / REPULSION_DECAY);
                v_rep = v_rep.plus(diff.normalize().times(scale));
            }
        }

        Vector2d finalVel = v_t.plus(v_p).plus(v_rep);
        double targetMag = power * curr.power;
        finalVel = finalVel.normalize().times(targetMag);

        Vector2d accel = spline.getAccel(t);
        double radius = spline.getR(t);

        return new PathData(finalVel, accel, radius, curr.power, heading, curr.reversed, index);
    }
}
