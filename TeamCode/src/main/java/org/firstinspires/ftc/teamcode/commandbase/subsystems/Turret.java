package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.ANGLE_CONTROL;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.GOAL_LOCK_CONTROL;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.TX_CONTROL;
import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.SquIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.ArrayList;

public class Turret extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    private final ElapsedTime timer = new ElapsedTime();
    private Pose2d turretPose = null;
    private final ArrayList<Pose2d> turretPoseEstimates = new ArrayList<>();

    public enum TurretState {
        TX_CONTROL,
        ANGLE_CONTROL,
        GOAL_LOCK_CONTROL,
        OFF,
    }

    public static TurretState turretState = ANGLE_CONTROL;
    public PIDFController turretController = new PIDFController(TURRET_PIDF_COEFFICIENTS);

    public Turret() {
        turretController.setOpenF(TURRET_OPEN_F);
        turretController.setTolerance(TURRET_POS_TOLERANCE);
        turretController.setMinOutput(0);
        turretController.setIntegrationControl(new PIDFController.IntegrationControl(TURRET_INTEGRATION_BEHAVIOR, TURRET_INTEGRATION_DECAY, TURRET_MIN_INTEGRAL, TURRET_MAX_INTEGRAL));
    }

    public void init() {
        if (!OP_MODE_TYPE.equals(OpModeType.AUTO)) {
            setTurret(TurretState.OFF, 0);
        }
    }

    public Pose2d getTurretPose() {
        if (turretPose == null) {
            turretPose = robot.drive.getPose();
        }
        return turretPose;
    }

    public void updateTurretPose(Pose2d turretPose) {
        this.turretPose = turretPose;
    }

    public void clearTurretPose() {
        turretPoseEstimates.clear();
        turretPose = null;
    }

    public void setTurret(TurretState turretState, double value) {
        turretController.setIntegrationControl(new PIDFController.IntegrationControl(TURRET_INTEGRATION_BEHAVIOR, TURRET_INTEGRATION_DECAY, TURRET_MIN_INTEGRAL, TURRET_MAX_INTEGRAL));
        turretController.setOpenF(TURRET_OPEN_F);

        switch (turretState) {
            case ANGLE_CONTROL:
                turretController.setTolerance(TURRET_POS_TOLERANCE, TURRET_VEL_TOLERANCE);
                turretController.setCoefficients(TURRET_PIDF_COEFFICIENTS);
                turretController.setMaxOutput(TURRET_LARGE_MAX_OUTPUT);
                turretController.setMinOutput(0);

                turretController.setSetPoint(Range.clip(value, -MAX_TURRET_ANGLE, MAX_TURRET_ANGLE));
                break;
            case GOAL_LOCK_CONTROL:
                turretController.setTolerance(TURRET_POS_TOLERANCE, TURRET_VEL_TOLERANCE);
                turretController.setCoefficients(TURRET_PIDF_COEFFICIENTS);
                turretController.setMaxOutput(TURRET_LARGE_MAX_OUTPUT);
                turretController.setMinOutput(0);

                getTurretPose(); // fixes goofy jinu
                double[] driveTurretErrors = Turret.angleToDriveTurretErrors(posesToAngle(turretPose, adjustedGoalPose(turretPose)));
                turretController.setSetPoint(driveTurretErrors[0] + driveTurretErrors[1]);

                break;
            case TX_CONTROL:
                turretController.setTolerance(CAMERA_TX_TOLERANCE, Double.POSITIVE_INFINITY);
                turretController.setCoefficients(CAMERA_PIDF_COEFFICIENTS);
                turretController.setMaxOutput(CAMERA_MAX_OUTPUT);
                turretController.setMinOutput(TURRET_MIN_OUTPUT);

                turretController.setSetPoint(value);
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
        return MathUtils.normalizeRadians(robot.turretEncoder.getCurrentPosition(), false);
    }

    public void update() {
        robot.turret.turretController.setOpenF(TURRET_OPEN_F);
        turretController.setIntegrationControl(new PIDFController.IntegrationControl(TURRET_INTEGRATION_BEHAVIOR, TURRET_INTEGRATION_DECAY, TURRET_MIN_INTEGRAL, TURRET_MAX_INTEGRAL));

        if (CommandScheduler.getInstance().isAvailable(robot.turret)) {
            clearTurretPose();
        }

        double power;
        switch (turretState) {
            case ANGLE_CONTROL:
                turretController.setTolerance(TURRET_POS_TOLERANCE, TURRET_VEL_TOLERANCE);
                turretController.setCoefficients(TURRET_PIDF_COEFFICIENTS);
                turretController.setMaxOutput(TURRET_LARGE_MAX_OUTPUT);

                robot.profiler.start("Turret Read");
                power = turretController.calculate(getPosition());
                robot.profiler.end("Turret Read");

                if (Math.abs(turretController.getPositionError()) > TURRET_POS_THRESHOLD) {
                    turretController.setMaxOutput(TURRET_LARGE_MAX_OUTPUT);
                } else {
                    turretController.setMaxOutput(TURRET_SMALL_MAX_OUTPUT);
                }

                robot.profiler.start("Turret Write");
                if (robot.turret.turretController.atSetPoint()) {
                    robot.turretServos.set(0);
                } else {
                    robot.turretServos.set(power);
                }
                robot.profiler.end("Turret Write");

                break;
            case GOAL_LOCK_CONTROL:
                turretController.setTolerance(TURRET_POS_TOLERANCE, TURRET_VEL_TOLERANCE);
                turretController.setCoefficients(TURRET_PIDF_COEFFICIENTS);
                turretController.setMaxOutput(TURRET_LARGE_MAX_OUTPUT);

                robot.profiler.start("Turret Read");

                double[] driveTurretErrors = Turret.angleToDriveTurretErrors(posesToAngle(getTurretPose(), adjustedGoalPose(getTurretPose())));

                turretController.setSetPoint(driveTurretErrors[0] + driveTurretErrors[1]);

                if (Math.abs(turretController.getPositionError()) > TURRET_POS_THRESHOLD) {
                    turretController.setMaxOutput(TURRET_LARGE_MAX_OUTPUT);
                } else {
                    turretController.setMaxOutput(TURRET_SMALL_MAX_OUTPUT);
                }

                if (robot.turret.turretController.atSetPoint()) {
                    robot.turretServos.set(0);
                    robot.turret.turretController.clearTotalError();
                }

                power = turretController.calculate(getPosition());
                robot.profiler.end("Turret Read");

                robot.profiler.start("Turret Write");
                if (Math.abs(getPosition()) < Math.abs(MAX_TURRET_ANGLE)) {
                    robot.turretServos.set(power);
                    RobotLog.aa("turret power", String.valueOf(power));
                } else {
                    robot.turretServos.set(0);
                }
                robot.profiler.end("Turret Write");
                break;
            case TX_CONTROL:
                robot.profiler.start("Turret Read");
                robot.camera.updateCameraResult(3);
                robot.profiler.end("Turret Read");

                turretController.setTolerance(CAMERA_TX_TOLERANCE, Double.POSITIVE_INFINITY);
                turretController.setPIDF(CAMERA_PIDF_COEFFICIENTS.p, CAMERA_PIDF_COEFFICIENTS.i, CAMERA_PIDF_COEFFICIENTS.d, CAMERA_PIDF_COEFFICIENTS.f);
                turretController.setMinOutput(CAMERA_MIN_OUTPUT);
                turretController.setMaxOutput(CAMERA_MAX_OUTPUT);

                double[] targetDegrees = robot.camera.getTargetDegrees(); // just to make sure reading is valid

                if (targetDegrees != null) {
                    double tx = targetDegrees[0];
                    power = -turretController.calculate(tx); // TODO: maybe figure out why we need to reverse power on TX_CONTROL
//                    power += robot.turretEncoder.getVelocity() * CAMERA_PIDF_COEFFICIENTS.d;

                    robot.profiler.start("Turret Write");
                    if (Math.abs(getPosition()) < Math.abs(MAX_TURRET_ANGLE)) {
                        robot.turretServos.set(power);
                    } else {
                        robot.turretServos.set(0);
                    }
                    robot.profiler.end("Turret Write");
                } else {
                    robot.profiler.start("Turret Write");
                    robot.turretServos.set(0); // turn off servo power if nothing is visible
                    robot.profiler.end("Turret Write");
                }
                break;
        }
    }

    public boolean readyToLaunch() {
        return turretController.atSetPoint() &&
                ((turretState.equals(ANGLE_CONTROL) || turretState.equals(GOAL_LOCK_CONTROL))
                  || (robot.camera.detections != null && turretController.atSetPoint() && turretState.equals(TX_CONTROL))
                );
    }

    public void updateTurretPoseReadings(Pose2d llPose) {
        turretPoseEstimates.add(llPose);

        if (turretPoseEstimates.size() > 10) {
            turretPoseEstimates.remove(0);
        }

        double avgX = turretPoseEstimates.stream()
                .mapToDouble(Pose2d::getX)
                .average()
                .orElse(llPose.getX());

        double avgY = turretPoseEstimates.stream()
                .mapToDouble(Pose2d::getY)
                .average()
                .orElse(llPose.getY());

        double avgHeading = turretPoseEstimates.stream()
                .mapToDouble(Pose2d::getHeading)
                .average()
                .orElse(llPose.getHeading());

        // Update the turretPose variable with the averaged values
        Pose2d newTurretPose = new Pose2d(avgX, avgY, avgHeading);
        if (turretPose == null
            || (turretPose.minus(newTurretPose).getTranslation().getNorm() > 2
                || turretPose.minus(newTurretPose).getRotation().getAngle(AngleUnit.RADIANS) > 0.1)) {
            robot.turret.updateTurretPose(newTurretPose);
        }
        timer.reset();
    }

    public double angleToWall(Pose2d robotPose) {
        return posesToAngle(new Pose2d(0, 72, 0), Constants.GOAL_POSE()) - posesToAngle(robotPose, Constants.GOAL_POSE());
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

    public Pose2d adjustedGoalPose(Pose2d robotPose) {
        if (turretPose == null) {
            turretPose = robotPose;
            updateTurretPose(robotPose);
        }

        if (robot.camera.medianWallAngle.isEmpty()) {
            robot.camera.updateMedianReadings(robotPose);
        }

        double offset = -robot.camera.getMedianWallAngle() * ALLIANCE_COLOR.getMultiplier();
        RobotLog.aa("offset", String.valueOf(offset));
        double adjustment = robot.camera.goalAdjustmentLUT.get(offset);
        RobotLog.aa("adjustment", String.valueOf(adjustment));

        Pose2d adjustedGoal;
        if (adjustment < 0) {
            adjustedGoal = new Pose2d(GOAL_POSE().getX() - (adjustment * ALLIANCE_COLOR.getMultiplier()), GOAL_POSE().getY(), GOAL_POSE().getHeading());
        } else {
            adjustedGoal = new Pose2d(GOAL_POSE().getX(), GOAL_POSE().getY() - adjustment, GOAL_POSE().getHeading());
        }

        RobotLog.aa("adjustedGoal", adjustedGoal.toString());

        return adjustedGoal;
    }

    public Pose2d adjustedGoalPose() {
        return adjustedGoalPose(robot.turret.getTurretPose());
    }

    /**
     * Converts an angle in radians (field-centric, normalized to 0-2pi) to two separate
     * robot-centric error angles, one for the drivetrain and one for the turret.
     * @param targetAngle The field-centric target angle in radians.
     * @return double[] {drivetrainError, turretError} (Robot-centric radians)
     */
    public static double[] angleToDriveTurretErrors(double targetAngle) {
        final double MAX_USABLE_TURRET_ANGLE = MAX_TURRET_ANGLE - TURRET_BUFFER;

        double robotAngle = Robot.getInstance().drive.getPose().getHeading();

        // 1. Calculate the total required rotation to face target (Robot Centric)
        double totalError = MathUtils.normalizeRadians(targetAngle - robotAngle, false);

        // 2. Logic Distribution
        double drivetrainError;
        double turretError;

        if (Math.abs(totalError) <= MAX_USABLE_TURRET_ANGLE) {
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
            turretError = MAX_USABLE_TURRET_ANGLE * sign;

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
