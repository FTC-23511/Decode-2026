package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.*;
import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Twist2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.MathFunctions;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.ArrayList;
import java.util.Arrays;

public class Turret extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public enum TurretState {
        GOAL_LOCK_CONTROL,
        ANGLE_CONTROL,
        OFF,
    }

    public final InterpLUT goalAdjustmentLUT = new InterpLUT(
            Arrays.asList(-Math.PI/2, -0.94, -0.9, -Math.PI/4, -0.6, -0.5, -0.3, -0.1,  0.25), // input: angle (radians) formed by lines between robot to goal and far field wall
            Arrays.asList(-3.0,       -3.0,  -3.0,  0.0,        1.67, 4.67, 6.67, 9.41, 9.41), // output: new goal pos (inches)
            true
    );

    private Pose2d turretPose = null;
    public static TurretState turretState = GOAL_LOCK_CONTROL;
    public PIDFController turretController = new PIDFController(TURRET_LARGE_PIDF_COEFFICIENTS);

//    public CascadeController turretController = new CascadeController(
//            new PIDFController(TURRET_LARGE_PIDF_COEFFICIENTS)
//                    .setIntegrationControl(
//                            new PIDFController.IntegrationControl(TURRET_INTEGRATION_BEHAVIOR, TURRET_INTEGRATION_DECAY, TURRET_MIN_INTEGRAL, TURRET_MAX_INTEGRAL)
//                    ),
//            new PIDFController(TURRET_SMALL_PIDF_COEFFICIENTS)
//                    .setIntegrationControl(
//                            new PIDFController.IntegrationControl(TURRET_INTEGRATION_BEHAVIOR, TURRET_INTEGRATION_DECAY, TURRET_MIN_INTEGRAL, TURRET_MAX_INTEGRAL)
//                    )
//    );

    private final ElapsedTime timer = new ElapsedTime();

    public Turret() {
        goalAdjustmentLUT.createLUT();
    }

    public void init() {
        if (!TESTING_OP_MODE) {
            setTurret(GOAL_LOCK_CONTROL, 0);
            ENABLE_ZONE_CONTROL = true;
        } else {
            setTurret(TurretState.OFF, 0);
            ENABLE_ZONE_CONTROL = false;
        }
    }

    public void resetTurretEncoder() {
        if (!TURRET_SYNCED) {
            if (robot.analogTurretEncoder.getVoltage() > 0.001) {
                robot.turretEncoder.overrideResetPos(0);
                TURRET_SYNC_OFFSET = robot.turretEncoder.getPosition() - (MathUtils.normalizeRadians(robot.analogTurretEncoder.getCurrentPosition(), false) / TURRET_RADIANS_PER_TICK);
                robot.turretEncoder.overrideResetPos((int) TURRET_SYNC_OFFSET);
                TURRET_SYNCED = true;
            }
        }
    }

    public Pose2d getTurretPose() {
        turretPose = robot.drive.getPose();
        return turretPose;
    }

    public void updateCoefficients() {
        turretController.setTolerance(TURRET_POS_TOLERANCE);
        turretController.setMaxOutput(TURRET_LARGE_MAX_OUTPUT);
//        turretController.setIntegrationControl(new PIDFController.IntegrationControl(TURRET_INTEGRATION_BEHAVIOR, TURRET_INTEGRATION_DECAY, TURRET_MIN_INTEGRAL, TURRET_MAX_INTEGRAL));
        turretController.setMinOutput(TURRET_MIN_OUTPUT);
    }

    public void setTurret(TurretState turretState, double value) {
        switch (turretState) {
            case ANGLE_CONTROL:
                setTurretPos(value);
        }

        Turret.turretState = turretState;
    }

    public void setTurretPos(double value, boolean useRawPos) {
        if (!useRawPos) {
            value = Range.clip(value, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);
            value = MathFunctions.convertRadianToServoPos(value);
        }

        value = Range.clip(value + TURRET_SERVO_OFFSET, 0, 1);
//        value = Range.clip(value + TURRET_SERVO_OFFSET, MathFunctions.convertRadianToServoPos(MIN_TURRET_ANGLE), MathFunctions.convertRadianToServoPos(MAX_TURRET_ANGLE));
        robot.turretServos.set(value);
    }

    public void setTurretPos(double angle) {
        setTurretPos(angle, false);
    }

    public double getTarget() {
        return turretController.getSetPoint();
    }

    public double getPosition() {
        if (!TURRET_SYNCED) {
            resetTurretEncoder();
        }
        return MathUtils.normalizeRadians(robot.turretEncoder.getPosition() * TURRET_RADIANS_PER_TICK, false);
    }

    public void update() {
        switch (turretState) {
            case GOAL_LOCK_CONTROL:
                robot.profiler.end("Turret Write");
                if (TESTING_OP_MODE) { // let the user "hack" the mode and take over what the turret is actually doing

                } else if (Drive.robotInZone(robot.drive.getPose()) && ENABLE_ZONE_CONTROL) {
                    // otherwise figure out what those pos and vel setpoints need to be
                    double[] driveTurretErrors = Turret.angleToDriveTurretErrors(posesToAngle(getTurretPose(), adjustedGoalPose()));
                    double setPoint = driveTurretErrors[0] + driveTurretErrors[1];

                    setTurretPos(setPoint);
                }

                robot.profiler.end("Turret Write");
                break;

            case ANGLE_CONTROL:
                break;
            case OFF:
                // We already set turret power to 0, so do nothing in the update
                break;
        }
    }

    public boolean readyToLaunch() {
        double servoPos = MathUtils.normalizeRadians(MathFunctions.convertServoPoseToRadian(robot.turretServos.get()), false);
        if (Double.isNaN(servoPos)) {
            return false;
        }

        return (Math.abs(getPosition() - servoPos) <= TURRET_POS_TOLERANCE) && !turretState.equals(OFF);
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
        double adjustment = goalAdjustmentLUT.get(offset);

        Pose2d adjustedGoal;
        if (adjustment < 0) {
            adjustedGoal = new Pose2d(GOAL_POSE().getX() - (adjustment * ALLIANCE_COLOR.getMultiplier()), GOAL_POSE().getY(), GOAL_POSE().getHeading());
        } else {
            adjustedGoal = new Pose2d(GOAL_POSE().getX(), GOAL_POSE().getY() - adjustment, GOAL_POSE().getHeading());
        }

        RobotLog.vv("Adjusted Goal Pose", adjustedGoal.toString());

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

        if (totalError <= MAX_TURRET_ANGLE && totalError >= MIN_TURRET_ANGLE) {
            // Target is within the usable turret range:
            // Drivetrain stays still, Turret handles the entire rotation.
            drivetrainError = 0;
            turretError = totalError;
        } else {
            // Target is outside the usable turret range:
            // Turret locks to its maximum usable angle in the direction of the target.
            // Drivetrain handles the remaining overflow (the amount needed to move the target
            // back into the turret's MAX range).

            if (totalError > MAX_TURRET_ANGLE) {
                turretError = MAX_TURRET_ANGLE;
            } else {
                turretError = MIN_TURRET_ANGLE;
            }

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
