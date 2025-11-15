package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.ANGLE_CONTROL;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.LIMELIGHT_CONTROL;
import static org.firstinspires.ftc.teamcode.globals.Constants.*;
import static org.firstinspires.ftc.teamcode.globals.Constants.GOAL_POSE;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.Arrays;

public class Turret extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    public final InterpLUT limelightInterplut = new InterpLUT(
            Arrays.asList(0.0, 8.40), // input: angle of 2 lines (degrees)
            Arrays.asList(0.0, 6.0) // output: new goal pos (inches)
    );

    public enum Motif {
        NOT_FOUND,
        GPP,
        PGP,
        PPG
    }

    public enum TurretState {
        LIMELIGHT_CONTROL,
        ANGLE_CONTROL,
        OFF
    }

    public static Motif motifState = Motif.NOT_FOUND;
    public static TurretState turretState = ANGLE_CONTROL;
    public LLResult llResult = null;
    public PIDFController turretController = new PIDFController(TURRET_PIDF_COEFFICIENTS);

    public Turret() {
        turretController.setMinimumOutput(TURRET_MIN_OUTPUT);
        turretController.setTolerance(TURRET_POS_TOLERANCE);
    }

    public void init() {
        setTurret(TurretState.OFF, 0);
    }

    public void setTurret(TurretState turretState, double value) {
        switch (turretState) {
            case ANGLE_CONTROL:
                turretController.setTolerance(TURRET_POS_TOLERANCE);
                turretController.setCoefficients(TURRET_PIDF_COEFFICIENTS);

                turretController.setSetPoint(Range.clip(value, -MAX_TURRET_ANGLE, MAX_TURRET_ANGLE));
                break;
            case LIMELIGHT_CONTROL:
                turretController.setTolerance(TURRET_TY_TOLERANCE);
                turretController.setCoefficients(LIMELIGHT_LARGE_PIDF_COEFFICIENTS);

                turretController.setSetPoint(value);
                turretController.calculate(value - TURRET_TY_TOLERANCE - 0.1); // update the internal controller's PV to outside of allowed tolerance so that readyToLaunch() does not return true instantly
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
                } else {
                    turretController.setCoefficients(LIMELIGHT_SMALL_PIDF_COEFFICIENTS);
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

                    robot.limelight.updateRobotOrientation(MathUtils.normalizeDegrees(robot.drive.getPose().getRotation().getDegrees() + 90.0, false));
                    Pose3D botPose = llResult.getBotpose_MT2();

                    if (botPose != null) {
                        Position position = botPose.getPosition();

                        double x = DistanceUnit.INCH.fromMeters(position.x);
                        double y = DistanceUnit.INCH.fromMeters(position.y);;

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

    public Pose2d getLimelightPose() {
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

    public double tyOffset(@NonNull Pose2d robotPose, Pose2d goalPose) {
        double angleToGoal = angleToPose(robotPose, goalPose);
        double angleToATag = angleToPose(robotPose, APRILTAG_POSE());

        return angleToATag - angleToGoal;
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

    /**
     * @param robotPose what the targetPose is being compared to
     * @param targetPose what the robotPose is being compared to
     * @return angle in radians, field-centric, normalized to 0-2pi
     */
    public static double angleToPose(Pose2d robotPose, Pose2d targetPose) {
        return MathUtils.normalizeRadians(
                new Vector2d(targetPose).minus(new Vector2d(robotPose)).angle(),
                true
        );
    }

    public double getTyOffset(@NonNull Pose2d robotPose) {
        double offset = tyOffset(robotPose, GOAL_POSE());
        double adjustment = limelightInterplut.get(offset);

        Pose2d adjustedGoal;
        if (adjustment < 0) {
            adjustedGoal = new Pose2d(GOAL_POSE().getX() + adjustment, GOAL_POSE().getY(), GOAL_POSE().getHeading());
        } else {
            adjustedGoal = new Pose2d(GOAL_POSE().getX(), GOAL_POSE().getY() + adjustment, GOAL_POSE().getHeading());
        }

        return tyOffset(robotPose, adjustedGoal);
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
}
