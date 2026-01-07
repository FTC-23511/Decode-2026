package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.*;
import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.SquIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Transform2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.ArrayList;
import java.util.Arrays;

public class Turret extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    private Pose2d turretPose = null;
    public static Translation2d goalPoseOffset = new Translation2d();

    public enum TurretState {
        GOAL_LOCK_CONTROL,
        ANGLE_CONTROL,
        OFF,
    }

    public final InterpLUT goalAdjustmentLUT = new InterpLUT(
            Arrays.asList(-Math.PI/2, -0.94, -0.9, -Math.PI/4, -0.6, -0.5, -0.3, -0.1, 0.25), // input: angle formed by lines between robot to goal and far field wall
            Arrays.asList(-12.0,      -12.0,  0.0,  0.0,        1.67, 4.67, 6.67, 9.41, 9.41), // output: new goal pos (inches)
            true
    );

    public static TurretState turretState = GOAL_LOCK_CONTROL;
    public SquIDFController turretController = new SquIDFController(TURRET_PIDF_COEFFICIENTS);
    public static double targetVel = 0;

//    public CascadeController turretController = new CascadeController(
//            new PIDFController(TURRET_PIDF_COEFFICIENTS)
//                    .setIntegrationControl(
//                            new PIDFController.IntegrationControl(TURRET_INTEGRATION_BEHAVIOR, TURRET_INTEGRATION_DECAY, TURRET_MIN_INTEGRAL, TURRET_MAX_INTEGRAL)
//                    ),
//            new PIDFController(TURRET_PIDF_COEFFICIENTS)
//                    .setIntegrationControl(
//                            new PIDFController.IntegrationControl(TURRET_INTEGRATION_BEHAVIOR, TURRET_INTEGRATION_DECAY, TURRET_MIN_INTEGRAL, TURRET_MAX_INTEGRAL)
//                    )
//    );

    private ArrayList<Double> lastVelocities = new ArrayList<>();
    private final ElapsedTime timer = new ElapsedTime();
    private double vel = 0;
    private double lastPos = Double.NaN; // for velocity
    private double lastPosPos = Double.NaN; // for position

    public Turret() {
        updateCoefficients();
        goalAdjustmentLUT.createLUT();
    }

    public void init() {
        if (!TESTING_OP_MODE) {
            setTurret(GOAL_LOCK_CONTROL, 0);
        } else {
            setTurret(TurretState.OFF, 0);
        }
    }

    public Pose2d getTurretPose() {
        turretPose = robot.drive.getPose();
        return turretPose;
    }

    public void updateCoefficients() {
        turretController.setTolerance(TURRET_POS_TOLERANCE);
        turretController.setCoefficients(TURRET_PIDF_COEFFICIENTS);
        turretController.setMaxOutput(TURRET_LARGE_MAX_OUTPUT);
        turretController.setIntegrationControl(new PIDFController.IntegrationControl(TURRET_INTEGRATION_BEHAVIOR, TURRET_INTEGRATION_DECAY, TURRET_MIN_INTEGRAL, TURRET_MAX_INTEGRAL));
        turretController.setMinOutput(TURRET_MIN_OUTPUT);
        turretController.setOpenF(TURRET_OPEN_F * (DEFAULT_VOLTAGE / robot.getVoltage()));
    }

    public void setTurret(TurretState turretState, double value) {
        updateCoefficients();
        switch (turretState) {
            case GOAL_LOCK_CONTROL:
                double[] driveTurretErrors = Turret.angleToDriveTurretErrors(posesToAngle(getTurretPose(), adjustedGoalPose()));
                double setPoint = driveTurretErrors[0] + driveTurretErrors[1];
                turretController.setSetPoint(Range.clip(setPoint, -MAX_TURRET_ANGLE, MAX_TURRET_ANGLE));
                break;
            case ANGLE_CONTROL:
                // value = turret target (radians)
                turretController.clearTotalError();
                turretController.setSetPoint(Range.clip(value, -MAX_TURRET_ANGLE, MAX_TURRET_ANGLE));
                break;
            case OFF:
                robot.turretServos.set(0);
                break;
        }

        Turret.turretState = turretState;
    }

    public double getTarget() {
        return turretController.getSetPoint();
    }

    public double getPosition() {
        double newPos = MathUtils.normalizeRadians(robot.turretEncoder.getCurrentPosition(), false);
        if ((((Double) lastPosPos).isNaN()) || (Math.abs(newPos - lastPosPos) < TURRET_POS_FILTER)) {
            lastPosPos = newPos;
            return newPos;
        }
        return lastPosPos;
    }

    public void updateVelocity() {
        double position = getPosition();
        if (((Double) lastPos).isNaN()) {
            lastPos = position;
        }

        double rawVel = (position - lastPos) / timer.seconds();

        double oldAvgVel = lastVelocities.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);

        if (Math.abs(rawVel - oldAvgVel) < TURRET_VEL_FILTER) {
            lastVelocities.add(rawVel);
            if (lastVelocities.size() > TURRET_LAST_VEL_ENTRIES) {
                lastVelocities.remove(0);
            }
        }

        vel = lastVelocities.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);

        lastPos = position;
        timer.reset();

    }

    public double getVelocity() {
        return vel;
    }

    public void update() {
        turretController.setOpenF(TURRET_OPEN_F * (DEFAULT_VOLTAGE / robot.getVoltage()));
        double power;

        switch (turretState) {
            case GOAL_LOCK_CONTROL:
                robot.profiler.start("Turret Read");
                updateVelocity(); // do not remove, needed for good tracking while bot is moving

                if (TESTING_OP_MODE) { // let the user "hack" the mode and take over what the turret is actually doing
                    // assume turretController setpoint has been set and targetVel has also been set
                } else {
                    // otherwise figure out what those pos and vel setpoints need to be
                    double[] driveTurretErrors = Turret.angleToDriveTurretErrors(posesToAngle(getTurretPose(), adjustedGoalPose()));
                    double setPoint = driveTurretErrors[0] + driveTurretErrors[1];

                    turretController.setSetPoint(Range.clip(setPoint, -MAX_TURRET_ANGLE, MAX_TURRET_ANGLE));
                    targetVel = -robot.drive.swerve.getTargetVelocity().omegaRadiansPerSecond;
                }

                if (Math.abs(turretController.getPositionError()) > TURRET_THRESHOLD) {
                    turretController.setMaxOutput(TURRET_LARGE_MAX_OUTPUT);
                } else {
                    turretController.setMaxOutput(TURRET_SMALL_MAX_OUTPUT);
                }

                robot.profiler.start("tr1");

                power = turretController.calculate(getPosition()); // PIF positional control output
                double errorVel = targetVel - getVelocity(); // custom D with smoothed velocity output (part 1)
                power += errorVel * TURRET_EXTERNAL_D; // custom D with smoothed velocity output (part 2)

                if (turretController.atSetPoint()) {
//                    power = 0;
                }

                power += targetVel * TURRET_VEL_FF * (DEFAULT_VOLTAGE / robot.getVoltage()); // velocity feedforward output

                RobotLog.aa("turret power", String.valueOf(power));
                robot.profiler.end("tr1");
                robot.profiler.end("Turret Read");

                robot.profiler.start("Turret Write");
                if ((Math.abs(getPosition()) > MAX_TURRET_ANGLE) && (Math.signum(power) == Math.signum(getPosition()))) {
                    // don't push the turret even further in that direction if it is already past the hardware limits
                    robot.turretServos.set(0);
                } else {
                    robot.turretServos.set(power);
                }
                robot.profiler.end("Turret Write");
                break;

            case ANGLE_CONTROL:
                power = turretController.calculate(getPosition());

                robot.turretServos.set(power);
                break;

            case OFF:
                // We already set turret power to 0, so do nothing in the update
                break;
        }
    }

    public boolean readyToLaunch() {
        return turretController.atSetPoint() && !turretState.equals(OFF);
    }

    /**
     * @param robotPose what the targetPose is being compared to
     * @param targetPose what the robotPose is being compared to
     * @return angle in radians, field-centric, normalized to 0-2pi
     */
    public static double posesToAngle(Pose2d robotPose, Pose2d targetPose) {
        return MathUtils.normalizeRadians(
                new Vector2d(targetPose).minus(new Vector2d(robotPose)).angle(),
                true
        );
    }

    public double angleToWall(Pose2d robotPose) {
        return posesToAngle(new Pose2d(0, 72, 0), Constants.GOAL_POSE()) - posesToAngle(robotPose, Constants.GOAL_POSE());
    }

    public double angleToWall() {
        return angleToWall(getTurretPose());
    }

    public Pose2d adjustedGoalPose() {
        getTurretPose();

        double offset = -angleToWall(turretPose) * ALLIANCE_COLOR.getMultiplier();
        RobotLog.aa("offset", String.valueOf(offset));
        double adjustment = goalAdjustmentLUT.get(offset);
        RobotLog.aa("adjustment", String.valueOf(adjustment));

        Pose2d adjustedGoal;
        if (adjustment < 0) {
            adjustedGoal = new Pose2d(GOAL_POSE().getX() - (adjustment * ALLIANCE_COLOR.getMultiplier()), GOAL_POSE().getY(), GOAL_POSE().getHeading());
        } else {
            adjustedGoal = new Pose2d(GOAL_POSE().getX(), GOAL_POSE().getY() - adjustment, GOAL_POSE().getHeading());
        }

        adjustedGoal.plus(new Transform2d(goalPoseOffset, new Rotation2d()));

        RobotLog.aa("adjustedGoal", adjustedGoal.toString());

        return adjustedGoal;
    }

    /**
     * Converts an angle in radians (field-centric, normalized to 0-2pi) to two separate
     * robot-centric error angles, one for the drivetrain and one for the turret.
     * @param targetAngle The field-centric target angle in radians.
     * @return double[] {drivetrainError, turretError} (Robot-centric radians)
     */
    public static double[] angleToDriveTurretErrors(double targetAngle) {
        double robotAngle = Robot.getInstance().drive.getPose().getHeading();

        // 1. Calculate the total required rotation to face target (Robot Centric)
        double totalError = MathUtils.normalizeRadians(targetAngle - robotAngle, false);

        // 2. Logic Distribution
        double drivetrainError;
        double turretError;

        if (Math.abs(totalError) <= MAX_TURRET_ANGLE) {
            // Target is within the usable turret range:
            // Drivetrain stays still, Turret handles the entire rotation.
            drivetrainError = 0;
            turretError = totalError;
        } else {
            // Target is outside the usable turret range:
            // Turret locks to its maximum usable angle in the direction of the target.
            // Drivetrain handles the remaining overflow (the amount needed to move the target
            // back into the turret's MAX range).

            double sign = Math.signum(totalError);

            // Turret locks to its maximum limit in the correct direction
            turretError = MAX_TURRET_ANGLE * sign;

            // Drivetrain takes the remainder: Total required - What the turret is doing
            // This is the essential fix from your original code—we return the ERROR delta.
            drivetrainError = totalError - turretError;
        }

        return new double[]{drivetrainError, turretError};
    }

    @Override
    public void periodic() {
        update();
    }
}
