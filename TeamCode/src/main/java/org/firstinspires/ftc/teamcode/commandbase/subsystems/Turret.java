package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.ANGLE_CONTROL;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.GOAL_LOCK_CONTROL;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.LIMELIGHT_CONTROL;
import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SubsystemBase;
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
        LIMELIGHT_CONTROL,
        ANGLE_CONTROL,
        GOAL_LOCK_CONTROL,
        OFF,
    }

    public static TurretState turretState = ANGLE_CONTROL;
    public SquIDFController turretController = new SquIDFController(TURRET_PIDF_COEFFICIENTS);


    public Turret() {
        turretController.setMinOutput(TURRET_MIN_OUTPUT);
        turretController.setTolerance(TURRET_POS_TOLERANCE);
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

    public void setTurret(TurretState turretState, double value) {
        switch (turretState) {
            case ANGLE_CONTROL:
                turretController.setTolerance(TURRET_POS_TOLERANCE, TURRET_VEL_TOLERANCE);
                turretController.setCoefficients(TURRET_PIDF_COEFFICIENTS);
                turretController.setMaxOutput(TURRET_LARGE_MAX_OUTPUT);
                turretController.integrationControl.setIntegrationBounds(TURRET_MIN_INTEGRAL, TURRET_MAX_INTEGRAL);

                turretController.setSetPoint(Range.clip(value, -MAX_TURRET_ANGLE, MAX_TURRET_ANGLE));
                break;
            case GOAL_LOCK_CONTROL:
                turretController.setTolerance(TURRET_POS_TOLERANCE, TURRET_VEL_TOLERANCE);
                turretController.setCoefficients(TURRET_PIDF_COEFFICIENTS);
                turretController.setMaxOutput(TURRET_LARGE_MAX_OUTPUT);
                turretController.integrationControl.setIntegrationBounds(TURRET_MIN_INTEGRAL, TURRET_MAX_INTEGRAL);

                getTurretPose(); // fixes goofy jinu
                double[] driveTurretErrors = Turret.angleToDriveTurretErrors(posesToAngle(turretPose, adjustedGoalPose(turretPose)));
                turretController.setSetPoint(driveTurretErrors[0] + driveTurretErrors[1]);

                break;
            case LIMELIGHT_CONTROL:
                turretController.setTolerance(TURRET_TY_TOLERANCE, Double.POSITIVE_INFINITY);
                turretController.setCoefficients(LIMELIGHT_LARGE_PIDF_COEFFICIENTS);
                turretController.setMaxOutput(LIMELIGHT_LARGE_MAX_OUTPUT);

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
        double power;
        switch (turretState) {
            case ANGLE_CONTROL:
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
                    robot.turret.turretController.clearTotalError();
                } else {
                    robot.turretServos.set(power);
                }
                robot.profiler.end("Turret Write");

                break;
            case GOAL_LOCK_CONTROL:
                robot.profiler.start("Turret Read");
                // only use drive pose estimate if we aren't in a command using the turret
                if (CommandScheduler.getInstance().isAvailable(robot.turret)) {
                    updateTurretPose(robot.drive.getPose());
                    turretPoseEstimates.clear();
                }

                double[] driveTurretErrors = Turret.angleToDriveTurretErrors(posesToAngle(turretPose, adjustedGoalPose(turretPose)));

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
            case LIMELIGHT_CONTROL:
                robot.profiler.start("Turret Read");
                robot.camera.updateLLResult(5);
                robot.profiler.end("Turret Read");

                double[] targetDegrees = robot.camera.getLimeLightTargetDegrees();

                if (targetDegrees != null) {
                    double ty = targetDegrees[1];
                    double error = ty - turretController.getSetPoint();

                    if (Math.abs(error) > LIMELIGHT_PID_THRESHOLD) {
                        turretController.setCoefficients(LIMELIGHT_LARGE_PIDF_COEFFICIENTS);
                        turretController.setMaxOutput(LIMELIGHT_LARGE_MAX_OUTPUT);
                    } else {
                        turretController.setCoefficients(LIMELIGHT_SMALL_PIDF_COEFFICIENTS);
                        turretController.setMaxOutput(LIMELIGHT_SMALL_MAX_OUTPUT);
                    }

                    power = turretController.calculate(ty);

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
        return (turretController.atSetPoint() && (turretState.equals(ANGLE_CONTROL) || turretState.equals(GOAL_LOCK_CONTROL)))
                || (robot.camera.llResult != null && turretController.atSetPoint() && turretState.equals(LIMELIGHT_CONTROL));
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
        if (turretPose.minus(newTurretPose).getTranslation().getNorm() > 2
            || turretPose.minus(newTurretPose).getRotation().getAngle(AngleUnit.RADIANS) > 0.25) {
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
        double adjustment = robot.camera.limelightInterplut.get(offset);
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
     * Converts an angle in radians, field-centric, normalized to 0-2pi to two separate angles, one for drivetrain and one for the turret
     * @param angle the angle to be converted
     * @return a two-item list of angles for the drivetrain and turret error in that specific order (robot-centric)
     */
    public static double[] angleToDriveTurretErrors(double angle) {
        final double MAX_USABLE_TURRET_ANGLE = MAX_TURRET_ANGLE - TURRET_BUFFER;
        Pose2d robotPose = Robot.getInstance().drive.getPose();
        double robotAngle = robotPose.getHeading();

        double error = MathUtils.normalizeRadians(angle - robotAngle, false);
        double localTurretBS = TURRET_BS + (robotPose.getY() < -36 ? 0.0441 : 0);
        if (Math.abs(error) < MAX_USABLE_TURRET_ANGLE) {
            return new double[]{0, error - localTurretBS};
        } else {
            return new double[]{robotAngle + (Math.abs(error) - MAX_USABLE_TURRET_ANGLE) * Math.signum(error), MAX_USABLE_TURRET_ANGLE * Math.signum(error) - localTurretBS};
        }
    }

    @Override
    public void periodic() {
        update();
    }
}
