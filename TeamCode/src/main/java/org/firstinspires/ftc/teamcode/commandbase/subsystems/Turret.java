package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.ACTIVE_TRACKING;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.ANGLE_CONTROL;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.LIMELIGHT_CONTROL;
import static org.firstinspires.ftc.teamcode.globals.Constants.*;
import static org.firstinspires.ftc.teamcode.globals.Constants.GOAL_POSE;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

public class Turret extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    public final InterpLUT limelightInterplut = new InterpLUT(
            Arrays.asList(-0.25, 0.1, 0.3,  0.5,  0.6,  Math.PI/4, 0.9, 0.94,   Math.PI/2), // input: angle formed by lines between robot to goal and far field wall
            Arrays.asList(7.0,   6.7, 5.67, 2.67, 1.67, 0.0,       0.0, -12.67, -12.67) // output: new goal pos (inches)
    );
    public Pose2d lastKnownPose = new Pose2d(); // only for logging purposes

    public enum Motif {
        NOT_FOUND,
        GPP,
        PGP,
        PPG
    }

    public enum TurretState {
        LIMELIGHT_CONTROL,
        ANGLE_CONTROL,
        ACTIVE_TRACKING,
        OFF
    }

    public static Motif motifState = Motif.NOT_FOUND;
    public static TurretState turretState = ANGLE_CONTROL;
    public LLResult llResult = null;
    private final ElapsedTime turretTimer;
    public PIDFController turretController = new PIDFController(TURRET_PIDF_COEFFICIENTS);
    public ArrayList<Double> medianWallAngle = new ArrayList<>();

    public Turret() {
        turretController.setMinimumOutput(TURRET_MIN_OUTPUT);
        turretController.setTolerance(TURRET_POS_TOLERANCE);
        limelightInterplut.createLUT();
        turretTimer = new ElapsedTime();
    }

    public void init() {
        setTurret(TurretState.OFF, 0);
    }

    public void setTurret(TurretState turretState, double value) {
        switch (turretState) {
            case ANGLE_CONTROL:
                turretController.setTolerance(TURRET_POS_TOLERANCE);
                turretController.setCoefficients(TURRET_PIDF_COEFFICIENTS);
                turretController.setMaxOutput(1);

                turretController.setSetPoint(Range.clip(value, -MAX_TURRET_ANGLE, MAX_TURRET_ANGLE));
                break;
            case LIMELIGHT_CONTROL:
                turretController.setTolerance(TURRET_TY_TOLERANCE);
                turretController.setCoefficients(LIMELIGHT_LARGE_PIDF_COEFFICIENTS);
                turretController.setMaxOutput(LIMELIGHT_LARGE_TURRET_MAX_OUTPUT);

                turretController.setSetPoint(value);
                break;
            case ACTIVE_TRACKING:
                turretController.setTolerance(TURRET_TY_TOLERANCE);
                turretController.setCoefficients(TURRET_PIDF_COEFFICIENTS);
                turretController.setMaxOutput(1);
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
        if (turretState.equals(ANGLE_CONTROL)) {
            robot.profiler.start("Turret Read");
            double power = turretController.calculate(getPosition());
            robot.profiler.end("Turret Read");

            robot.profiler.start("Turret Write");
            robot.turretServos.set(power);
            robot.profiler.end("Turret Write");

        } else if (turretState.equals(LIMELIGHT_CONTROL)) {
            robot.profiler.start("Turret Read");
            updateLLResult(5);
            robot.profiler.end("Turret Read");

            double[] targetDegrees = getLimeLightTargetDegrees();

            if (targetDegrees != null) {
                double ty = targetDegrees[1];
                double error = ty - turretController.getSetPoint();

                if (Math.abs(error) > LIMELIGHT_PID_THRESHOLD) {
                    turretController.setCoefficients(LIMELIGHT_LARGE_PIDF_COEFFICIENTS);
                    turretController.setMaxOutput(LIMELIGHT_LARGE_TURRET_MAX_OUTPUT);
                } else {
                    turretController.setCoefficients(LIMELIGHT_SMALL_PIDF_COEFFICIENTS);
                    turretController.setMaxOutput(LIMELIGHT_SMALL_TURRET_MAX_OUTPUT);
                }

                double power = turretController.calculate(ty);

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
        } else if (turretState.equals(ACTIVE_TRACKING)) {
            robot.profiler.start("Turret Read");
            if (OP_MODE_TYPE.equals(OpModeType.AUTO) || turretTimer.milliseconds() > 1000 / LIMELIGHT_POLLING_RATE) {
                turretTimer.reset();
                updateLLResult(5);
            }
            robot.profiler.end("Turret Read");

            double[] targetDegrees = getLimeLightTargetDegrees();

            if (targetDegrees != null) {
                double ty = targetDegrees[1];
                double power = turretController.calculate(ty);

                robot.profiler.start("Turret Write");
                if (Math.abs(getPosition()) < Math.abs(MAX_TURRET_ANGLE)) {
                    robot.turretServos.set(power);
                } else {
                    robot.turretServos.set(0);
                }
                robot.profiler.end("Turret Write");
            } else {
                robot.profiler.start("Turret Write");
                double[] errorsDriveTurret = Turret.angleToDriveTurretErrors(Turret.posesToAngle(robot.drive.getPose(), GOAL_POSE()));
                robot.turret.setTurret(Turret.TurretState.ANGLE_CONTROL, errorsDriveTurret[1]);
                robot.profiler.end("Turret Write");
            }
        }
    }

    public boolean readyToLaunch() {
        return (turretController.atSetPoint() && turretState.equals(ANGLE_CONTROL))
                || (llResult != null && turretController.atSetPoint() && turretState.equals(LIMELIGHT_CONTROL));
    }

    public LLStatus getLimelightStatus() {
        return robot.limelight.getStatus();
    }

    /**
     * Updates internal limelight result in the Turret class
     * @param n max number of times to attempt reading to get a valid result
     */
    public void updateLLResult(int n) {
        llResult = null;

        for (int i = n; i > 0; i--) {
            llResult = robot.limelight.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                Pose2d llPose = getLimelightPose();

                if (llPose != null) {
                    updateMedianReadings(llPose);
                }

                if (medianWallAngle.size() > 10) {
                    medianWallAngle.remove(0);
                }
                break;
            } else {
                llResult = null;
            }
        }
    }

    public double[] getLimeLightTargetDegrees() {
        double[] targetDegrees = new double[2];

        if (llResult != null) {
            for (LLResultTypes.FiducialResult fiducial : llResult.getFiducialResults()) {
                int id = fiducial.getFiducialId();

                if ((Constants.ALLIANCE_COLOR.equals(Constants.AllianceColor.BLUE) && id == 20)
                        || (Constants.ALLIANCE_COLOR.equals(Constants.AllianceColor.RED) && id == 24)) {

                    targetDegrees[0] = fiducial.getTargetXDegrees();
                    targetDegrees[1] = fiducial.getTargetYDegrees();

                    return targetDegrees;
                }
            }
        }

        return null;
    }

    public Pose2d getLimelightPoseMT2() {
        if (llResult != null) {
            for (LLResultTypes.FiducialResult fiducial : llResult.getFiducialResults()) {
                int id = fiducial.getFiducialId();

                if ((Constants.ALLIANCE_COLOR.equals(Constants.AllianceColor.BLUE) && id == 20)
                        || (Constants.ALLIANCE_COLOR.equals(Constants.AllianceColor.RED) && id == 24)) {

                    robot.limelight.updateRobotOrientation(
                            MathUtils.normalizeDegrees(
                                    robot.drive.getPose().getRotation().getDegrees()
                                            + Math.toDegrees(getPosition()) - 90.0, false)
                    );

                    Pose3D botPose = llResult.getBotpose_MT2();

                    if (botPose != null) {
                        double x = DistanceUnit.INCH.fromMeters(botPose.getPosition().y);
                        double y = -DistanceUnit.INCH.fromMeters(botPose.getPosition().x);
                        double heading = botPose.getOrientation().getYaw(AngleUnit.RADIANS);
                        heading = MathUtils.normalizeRadians(heading + Math.PI / 2, true); // TODO: Figure out angle difference between our coordinate system and LL's

                        if (x > -80 && x < 80 && y > -80 && y < 80 && !Double.isNaN(heading)) {
                            return new Pose2d(x, y, heading);
                        }
                    }
                }
            }
        }

        return null;
    }

    public Pose2d getLimelightPoseMT1() {
        if (llResult != null) {
            for (LLResultTypes.FiducialResult fiducial : llResult.getFiducialResults()) {
                int id = fiducial.getFiducialId();

                if ((Constants.ALLIANCE_COLOR.equals(Constants.AllianceColor.BLUE) && id == 20)
                        || (Constants.ALLIANCE_COLOR.equals(Constants.AllianceColor.RED) && id == 24)) {

                    Pose3D botPose = llResult.getBotpose();

                    if (botPose != null) {
                        double x = DistanceUnit.INCH.fromMeters(botPose.getPosition().y);
                        double y = -DistanceUnit.INCH.fromMeters(botPose.getPosition().x);
                        double heading = botPose.getOrientation().getYaw(AngleUnit.RADIANS);

                        heading = MathUtils.normalizeRadians(heading + Math.PI/2, true); // TODO: Figure out angle difference between our coordinate system and LL's

                        if (x > -80 && x < 80 && y > -80 && y < 80 && !Double.isNaN(heading)) {
                            return new Pose2d(x, y, heading);
                        }
                    }
                }
            }
        }

        return null;
    }

    public Pose2d getLimelightPose() {
        return USE_LIMELIGHT_MT1 ? getLimelightPoseMT1() : getLimelightPoseMT2();
    }

    public void updateMedianReadings(Pose2d llPose) {
        medianWallAngle.add(posesToAngle(llPose, Constants.GOAL_POSE()) - posesToAngle(new Pose2d(0, 72, 0), Constants.GOAL_POSE()));
    }

    public double getMedianWallAngle() {
        if (medianWallAngle.isEmpty()) {
            return Math.PI / 4;
        }

        ArrayList<Double> sortedMedianWallAngle = new ArrayList<>(medianWallAngle);
        Collections.sort(sortedMedianWallAngle);

        if (sortedMedianWallAngle.size() % 2 == 1) {
            return sortedMedianWallAngle.get(sortedMedianWallAngle.size() / 2);
        }

        return
            (sortedMedianWallAngle.get(sortedMedianWallAngle.size() / 2 - 1) +
            sortedMedianWallAngle.get(sortedMedianWallAngle.size() / 2)) / 2.0;
    }

    public double tyOffset(@NonNull Pose2d robotPose, Pose2d goalPose) {
        double angleToGoal = Math.toDegrees(posesToAngle(robotPose, goalPose));
        double angleToATag = Math.toDegrees(posesToAngle(robotPose, APRILTAG_POSE()))   ;

        return angleToATag - angleToGoal;
    }

    public double getTyOffset(Pose2d robotPose) {
        if (robotPose == null) {
            return 0;
        }

        double offset = -getMedianWallAngle() * ALLIANCE_COLOR.getMultiplier();
        RobotLog.aa("offset", String.valueOf(offset));
        double adjustment = limelightInterplut.get(offset);
        RobotLog.aa("adjustment", String.valueOf(adjustment));

        Pose2d adjustedGoal;
        if (adjustment < 0) {
            adjustedGoal = new Pose2d(GOAL_POSE().getX() - (adjustment * ALLIANCE_COLOR.getMultiplier()), GOAL_POSE().getY(), GOAL_POSE().getHeading());
        } else {
            adjustedGoal = new Pose2d(GOAL_POSE().getX(), GOAL_POSE().getY() - adjustment, GOAL_POSE().getHeading());
        }

        double finalOffset = tyOffset(robotPose, adjustedGoal);
        RobotLog.aa("final offset", String.valueOf(finalOffset));
        return finalOffset;
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

    /**
     * Converts an angle in radians, field-centric, normalized to 0-2pi to two separate angles, one for drivetrain and one for the turret
     * @param angle the angle to be converted
     * @return a two-item list of angles for the drivetrain and turret error in that specific order (robot-centric)
     */
    public static double[] angleToDriveTurretErrors(double angle) {
        final double MAX_USABLE_TURRET_ANGLE = MAX_TURRET_ANGLE - TURRET_BUFFER;
        double robotAngle = Robot.getInstance().drive.getPose().getHeading();

        double error = MathUtils.normalizeRadians(angle - robotAngle, false);
        if (Math.abs(error) < MAX_USABLE_TURRET_ANGLE) {
            return new double[]{0, error};
        } else {
            return new double[]{robotAngle + (Math.abs(error) - MAX_USABLE_TURRET_ANGLE) * Math.signum(error), MAX_USABLE_TURRET_ANGLE * Math.signum(error)};
        }
    }

    @Override
    public void periodic() {
        update();
    }

    public boolean setMotifState() {
        if (llResult != null && llResult.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : llResult.getFiducialResults()) {
                int id = fiducial.getFiducialId();

                if (id == 21) {
                    motifState = Motif.PPG;
                    return true;
                } else if (id == 22) {
                    motifState = Motif.PGP;
                    return true;
                } else if (id == 23) {
                    motifState = Motif.GPP;
                    return true;
                }
            }
        }

        return false;
    }
}
