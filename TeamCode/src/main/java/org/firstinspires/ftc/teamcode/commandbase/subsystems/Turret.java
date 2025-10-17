package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.globals.Robot;

public class Turret extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    public static PIDFController turretController = new PIDFController(TURRET_PIDF_COEFFICIENTS);

    public void init() {
        setTarget(0);
    }

    public void setTarget(double target) {
        turretController.setSetPoint(target);
    }

    public double getTarget() {
        return turretController.getSetPoint();
    }

    public void update() {
        robot.turretServos.set(
                turretController.calculate(
                        MathUtils.normalizeRadians(robot.turretEncoder.getCurrentPosition(), false)
                )
        );
    }

    public boolean readyToLaunch() {
        return turretController.atSetPoint();
    }

    @Override
    public void periodic() {
        update();
    }

    /**
     * @param pose what the goal pose is being compared to
     * @return angle in radians, field-centric, normalized to 0-2pi
     */
    public static double angleToGoal(Pose2d pose) {
        return MathUtils.normalizeRadians(
                new Vector2d(GOAL_POSE()).minus(new Vector2d(pose)).angle(),
                true
        );
    }

    /**
     * Converts an angle in radians, field-centric, normalized to 0-2pi to two separate angles, one for drivetrain and one for the turret
     * @param angle the angle to be converted
     * @return a two-item list of angles for the drivetrain and turret error in that specific order (robot-centric)
     */
    public static double[] angleToDriveTurretErrors(double angle) {
        final double MAX_TURRET_BUFFER = 0.1; // Buffer angle from max angle of turret in radians
        final double MAX_USABLE_TURRET_ANGLE = MAX_TURRET_ANGLE - MAX_TURRET_BUFFER;
        double robotAngle = Robot.getInstance().drive.getPose().getHeading();

        double error = MathUtils.normalizeRadians(angle - robotAngle, false);
        if (Math.abs(error) < MAX_USABLE_TURRET_ANGLE) {
            return new double[]{0, error};
        } else {
            return new double[]{robotAngle + (Math.abs(error) - MAX_USABLE_TURRET_ANGLE) * Math.signum(error), MAX_USABLE_TURRET_ANGLE * Math.signum(error)};
        }
    }
}
