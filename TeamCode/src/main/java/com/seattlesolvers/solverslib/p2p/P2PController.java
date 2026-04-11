package com.seattlesolvers.solverslib.p2p;

import com.seattlesolvers.solverslib.controller.Controller;
import com.seattlesolvers.solverslib.gamepad.SlewRateLimiter;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Transform2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class P2PController {
    public final Controller translationalController;
    public final Controller headingController;
    public final AngleUnit angleUnit;

    private Pose2d target;
    private Pose2d current;

    private Transform2d error;

    private SlewRateLimiter magnitudeLimiter = null;
    private SlewRateLimiter hLimiter = null;

    public P2PController(Controller translationalController, Controller headingController, AngleUnit angleUnit, Pose2d start, Pose2d target, double positionalTolerance, double angularTolerance) {
        this.translationalController = translationalController;
        this.headingController = headingController;
        this.angleUnit = angleUnit;
        this.current = start;
        this.target = target;
        getError(); // updates error
        setTolerance(positionalTolerance, angularTolerance);
    }

    public P2PController(Controller translationalController, Controller headingController, AngleUnit angleUnit, double positionalTolerance, double angularTolerance) {
        this(translationalController, headingController, angleUnit, new Pose2d(), new Pose2d(), positionalTolerance, angularTolerance);
    }

    /**
     * The main method to calculate and return an output for robot movement
     * @param pv the last known position of the robot
     * @return field-centric chassis speeds/power
     */
    public ChassisSpeeds calculate(Pose2d pv) {
        current = pv;

        // 1. Calculate Global Coordinate Error (No rotation applied yet)
        double errorX = target.getX() - current.getX();
        double errorY = target.getY() - current.getY();

        // Update the error object just for telemetry use
        error = new Transform2d(new Pose2d(errorX, errorY, new Rotation2d(0)), new Pose2d());

        // 2. Vector Math
        double distanceToTarget = Math.hypot(errorX, errorY);
        double errorAngle = Math.atan2(errorY, errorX);

        // 3. Magnitude Calculation (PID on distance)
        // Setpoint is 0 (distance remaining), Measurement is current distance
        // NOTE: Depending on your PID logic, you might need to swap these or negate the result.
        // Usually: Output = Kp * (Setpoint - Measurement).
        // If Setpoint=0 and Measurement=10, Error=-10. Output is negative.
        // But we need positive magnitude to move forward.
        // So we calculate: target=0, input=distance -> returns negative?
        // EASIER WAY: calculate(distance, 0) -> Error = 0 - distance = -distance.
        // Let's assume calculate(current, target) -> Kp * (Target - Current).
        // calculate(0, distance) -> Kp * (distance - 0) -> Positive Magnitude.
        double magnitudeVal = translationalController.calculate(0, distanceToTarget);

        // 4. Slew Rate on Magnitude
        if (magnitudeLimiter != null) {
            magnitudeVal = magnitudeLimiter.calculate(magnitudeVal);
        }

        // 5. Decompose back to Field-Centric X/Y
        double xVal = magnitudeVal * Math.cos(errorAngle);
        double yVal = magnitudeVal * Math.sin(errorAngle);

        // 6. Heading Calculation
        double currentHeading = current.getRotation().getAngle(angleUnit);
        double targetHeading = target.getRotation().getAngle(angleUnit);

        // Use MathUtils to normalize the error to [-PI, PI] or [-180, 180]
        double headingError = MathUtils.normalizeAngle(targetHeading - currentHeading, false, angleUnit);

        // We feed 0 as current and error as target so the PID acts on the error directly
        double headingVal = headingController.calculate(0, headingError);

        if (hLimiter != null) {
            headingVal = hLimiter.calculate(headingVal);
        }

        return new ChassisSpeeds(xVal, yVal, headingVal);
    }

    public P2PController setSlewRateLimiters(SlewRateLimiter magnitudeLimiter, SlewRateLimiter hLimiter) {
        this.magnitudeLimiter = magnitudeLimiter;
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
        translationalController.setTolerance(positionTolerance);
        headingController.setTolerance(angularTolerance);
    }

    /**
     * @return the positional and angular tolerances of the controller respectively
     */
    public double[] getTolerance() {
        return new double[]{translationalController.getTolerance()[0], headingController.getTolerance()[0]};
    }

    /**
     * Returns true if the error is within the tolerance set by the user through {@link #setTolerance}.
     *
     * @return Whether the error is within the acceptable bounds.
     */
    public boolean atTarget() {
        return translationalController.atSetPoint() && headingController.atSetPoint();
    }

    /**
     * Updates the internal object for error and returns it
     *
     * @return the positional and angular error
     */
    public Transform2d getError() {
        // Re-calculate simply for accessors
        double errorX = target.getX() - current.getX();
        double errorY = target.getY() - current.getY();
        double errorH = MathUtils.normalizeAngle(target.getRotation().getAngle(angleUnit) - current.getRotation().getAngle(angleUnit), false, angleUnit);

        error = new Transform2d(new Pose2d(errorX, errorY, new Rotation2d(errorH)), new Pose2d());
        return error;
    }
}