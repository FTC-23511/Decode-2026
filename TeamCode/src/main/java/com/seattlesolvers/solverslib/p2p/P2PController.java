package com.seattlesolvers.solverslib.p2p;

import com.seattlesolvers.solverslib.controller.Controller;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.SlewRateLimiter;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Transform2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class P2PController {
    public final Controller xController;
    public final Controller yController;
    public final Controller headingController;
    public final AngleUnit angleUnit;

    private Pose2d target;
    private Pose2d current;
    private Transform2d error;

    private SlewRateLimiter xLimiter = null;
    private SlewRateLimiter yLimiter = null;
    private SlewRateLimiter hLimiter = null;

    /**
     * The constructor for a P2PController object.
     *
     * @param xController the controller for x axis movement (field-centric)
     * @param yController the controller for y axis movement (field-centric)
     * @param headingController the controller for robot heading
     * @param angleUnit the angle of the heading controller and positions being passed to this object
     * @param start the starting pose of the robot
     * @param target the first target pose of the robot
     * @param positionalTolerance the positional tolerance allowed for this controller at which the robot is considered to be at the target
     * @param angularTolerance the positional tolerance allowed for this controller at which the robot is considered to be at the target, in the units specified in the constructor
     */
    public P2PController(Controller xController, Controller yController, Controller headingController, AngleUnit angleUnit, Pose2d start, Pose2d target, double positionalTolerance, double angularTolerance) {
        this.xController = xController;
        this.yController = yController;
        this.headingController = headingController;
        this.angleUnit = angleUnit;
        this.current = start;
        this.target = target;
        getError(); // updates error
        setTolerance(positionalTolerance, angularTolerance);
    }

    /**
     * A simplified constructor for a P2PController object.
     *
     * @param xController the controller for x axis movement (field-centric)
     * @param yController the controller for y axis movement (field-centric)
     * @param headingController the controller for robot heading
     * @param angleUnit the angle of the heading controller and positions being passed to this object
     * @param positionalTolerance the positional tolerance allowed for this controller at which the robot is considered to be at the target
     * @param angularTolerance the positional tolerance allowed for this controller at which the robot is considered to be at the target, in the units specified in the constructor
     */
    public P2PController(Controller xController, Controller yController, Controller headingController, AngleUnit angleUnit, double positionalTolerance, double angularTolerance) {
        this(xController, yController, headingController, angleUnit, new Pose2d(), new Pose2d(), positionalTolerance, angularTolerance);
    }

    /**
     * The main method to calculate and return an output for robot movement
     * @param pv the last known position of the robot
     * @return field-centric chassis speeds/power
     */
    public ChassisSpeeds calculate(Pose2d pv) {
        // Update internal variables
        current = pv;
        getError();

        double xVal = xController.calculate(current.getX(), target.getX());
        double yVal = yController.calculate(current.getY(), target.getY());
        double headingVal = headingController.calculate(0, MathUtils.normalizeAngle(error.getRotation().getAngle(angleUnit), false, angleUnit));

        if (xLimiter != null) {
            xVal = xLimiter.calculate(xVal);
        }
        if (xLimiter != null) {
            yVal = yLimiter.calculate(yVal);
        }
        if (hLimiter != null) {
            headingVal = hLimiter.calculate(headingVal);
        }

        return new ChassisSpeeds(xVal, yVal, headingVal);
    }

    /**
     * Enables and sets the slew rate limiting for the P2P Controller.
     * One for x-axis movement, one for y-axis movement, and one for heading
     * Set any parameters not to be enabled/used as null.
     * @return this object for chaining purposes
     */
    public P2PController setSlewRateLimiters(SlewRateLimiter xLimiter, SlewRateLimiter yLimiter, SlewRateLimiter hLimiter) {
        this.xLimiter = xLimiter;
        this.yLimiter = yLimiter;
        this.hLimiter = hLimiter;
        return this;
    }

    /**
     * Sets the target pose
     *
     * @param sp The desired pose.
     */
    public void setTarget(Pose2d sp) {
        target = sp;
    }

    /**
     * @return The current target pose.
     */
    public Pose2d getTarget() {
        return target;
    }

    /**
     * Sets the error which is considered tolerable for use with {@link #atTarget()}.
     *
     * @param positionTolerance Position error which is tolerable.
     * @param angularTolerance Angular error which is tolerable, in the angle unit specified.
     */
    public void setTolerance(double positionTolerance, double angularTolerance) {
        xController.setTolerance(positionTolerance);
        yController.setTolerance(positionTolerance);
        headingController.setTolerance(angularTolerance);
    }

    /**
     * @return the positional and angular tolerances of the controller respectively
     */
    public double[] getTolerance() {
        return new double[]{xController.getTolerance()[0], headingController.getTolerance()[0]};
    }

    /**
     * Returns true if the error is within the tolerance set by the user through {@link #setTolerance}.
     *
     * @return Whether the error is within the acceptable bounds.
     */
    public boolean atTarget() {
        return xController.atSetPoint() && yController.atSetPoint() && headingController.atSetPoint();
    }

    /**
     * Updates the internal object for error and returns it
     *
     * @return the positional and angular error
     */
    public Transform2d getError() {
        error = target.minus(current);
        return error;
    }
}
