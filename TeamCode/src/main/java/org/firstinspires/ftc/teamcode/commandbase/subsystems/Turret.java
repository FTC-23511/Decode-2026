package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.ANGLE_CONTROL;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.LIMELIGHT_CONTROL;
import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

public class Turret extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

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
    public boolean ATagVisible = false;
    public static PIDFController turretController = new PIDFController(TURRET_PIDF_COEFFICIENTS);

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
                ATagVisible = false;
                turretController.setTolerance(TURRET_TY_TOLERANCE);
                turretController.setCoefficients(LIMELIGHT_LARGE_PIDF_COEFFICIENTS);

                turretController.setSetPoint(0);
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
            double[] targetDegrees = getLimeLightTargetDegrees();
            robot.profiler.end("Turret Read");

            if (targetDegrees != null) {
                double ty = targetDegrees[1];
                ATagVisible = true;

                if (Math.abs(ty) > LIMELIGHT_PID_THRESHOLD) {
                    turretController.setCoefficients(LIMELIGHT_LARGE_PIDF_COEFFICIENTS);
                } else {
                    turretController.setCoefficients(LIMELIGHT_SMALL_PIDF_COEFFICIENTS);
                }

                double power = turretController.calculate(ty);

                robot.profiler.start("Turret Write");
                robot.turretServos.set(power);
                robot.profiler.end("Turret Write");
            } else {
                ATagVisible = false;
                robot.profiler.start("Turret Write");
                robot.turretServos.set(0);
                robot.profiler.end("Turret Write");
            }
        } else {
            robot.profiler.start("Turret Write");
            robot.turretServos.set(0);
            robot.profiler.end("Turret Write");
        }
    }

    public boolean readyToLaunch() {
        return (turretController.atSetPoint() && turretState.equals(ANGLE_CONTROL))
                || (turretController.atSetPoint() && ATagVisible && turretState.equals(LIMELIGHT_CONTROL));
    }

    public LLStatus getLimelightStatus() {
        return robot.limelight.getStatus();
    }

    public double[] getLimeLightTargetDegrees() {
        double[] targetDegrees = new double[2];
        LLResult result = robot.limelight.getLatestResult();

        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
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

    /*
    public Pose2d getLimelightPoseMT2() {
        LLResult result = robot.limelight.getLatestResult();

        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                int id = fiducial.getFiducialId();

                if ((Constants.ALLIANCE_COLOR.equals(Constants.AllianceColor.BLUE) && id == 20)
                        || (Constants.ALLIANCE_COLOR.equals(Constants.AllianceColor.RED) && id == 24)) {

                    robot.limelight.updateRobotOrientation(robot.drive.getPose().getHeading());
                    Pose3D botPose = result.getBotpose_MT2();

                    if (botPose != null) {
                        double x = botPose.getPosition().x;
                        double y = botPose.getPosition().y;
                        double z = botPose.getPosition().z;

                        x = DistanceUnit.INCH.fromMeters(x);
                        y = DistanceUnit.INCH.fromMeters(y);
                        z = DistanceUnit.INCH.fromMeters(z);

                        if (x > -72 && x < 72 && y > -72 && y < 72 && !Double.isNaN(z)) {
                            return new Pose2d(x, y, z);
                        }
                    }
                }
            }
        }

        return null;
    }

     */

    public Pose2d getLimelightPose() {
        LLResult result = robot.limelight.getLatestResult();

        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                int id = fiducial.getFiducialId();

                if ((Constants.ALLIANCE_COLOR.equals(Constants.AllianceColor.BLUE) && id == 20)
                        || (Constants.ALLIANCE_COLOR.equals(Constants.AllianceColor.RED) && id == 24)) {

                    Pose3D botPose = result.getBotpose();

                    if (botPose != null) {
                        double x = botPose.getPosition().x;
                        double y = botPose.getPosition().y;
                        double z = botPose.getPosition().z;

                        x = DistanceUnit.INCH.fromMeters(x);
                        y = DistanceUnit.INCH.fromMeters(y);
                        z = DistanceUnit.INCH.fromMeters(z);

                        if (x > -72 && x < 72 && y > -72 && y < 72 && !Double.isNaN(z)) {
                            return new Pose2d(x, y, z);
                        }
                    }
                }
            }
        }

        return null;
    }

    public Pose2d getLimeLightPose(int n) {
        Pose2d pose2d = null;
        int m = n;

        while (pose2d == null && m > 0) {
            pose2d = getLimelightPose();
            m--;
        }
        return pose2d;
    }

    public boolean setMotifState() {
        LLResult result = robot.limelight.getLatestResult();

        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
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

    @Override
    public void periodic() {
        update();
    }
}
