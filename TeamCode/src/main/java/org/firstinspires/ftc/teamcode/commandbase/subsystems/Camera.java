package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.posesToAngle;
import static org.firstinspires.ftc.teamcode.globals.Constants.ALLIANCE_COLOR;
import static org.firstinspires.ftc.teamcode.globals.Constants.APRILTAG_POSE;
import static org.firstinspires.ftc.teamcode.globals.Constants.GOAL_POSE;
import static org.firstinspires.ftc.teamcode.globals.Constants.USE_LIMELIGHT_MT1;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.geometry.Pose2d;
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

public class Camera {
    private final Robot robot = Robot.getInstance();
    public static Motif motifState = Motif.NOT_FOUND;
    public LLResult llResult = null;
    public ArrayList<Double> medianWallAngle = new ArrayList<>();

    double bs = (ALLIANCE_COLOR.equals(Constants.AllianceColor.BLUE) ? -12.67 : -5.1);
    public final InterpLUT limelightInterplut = new InterpLUT(
            Arrays.asList(-Math.PI/2, -0.94, -0.9, -Math.PI/4, -0.6, -0.5, -0.3, -0.1, 0.25), // input: angle formed by lines between robot to goal and far field wall
            Arrays.asList( bs,         bs,    0.0,  0.0,        1.67, 4.67, 7.41, 14.14, 14.14) // output: new goal pos (inches)
    );

    public enum Motif {
        NOT_FOUND,
        GPP,
        PGP,
        PPG
    }

    public Camera() {
        limelightInterplut.createLUT();
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
                    robot.turret.updateTurretPoseReadings(llPose);
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

    public double tyOffset(@NonNull Pose2d robotPose, Pose2d goalPose) {
        double angleToGoal = Math.toDegrees(posesToAngle(robotPose, goalPose));
        double angleToATag = Math.toDegrees(posesToAngle(robotPose, APRILTAG_POSE()));

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

    private Pose2d getLimelightPoseMT2() {
        if (llResult != null) {
            for (LLResultTypes.FiducialResult fiducial : llResult.getFiducialResults()) {
                int id = fiducial.getFiducialId();

                if ((Constants.ALLIANCE_COLOR.equals(Constants.AllianceColor.BLUE) && id == 20)
                        || (Constants.ALLIANCE_COLOR.equals(Constants.AllianceColor.RED) && id == 24)) {

                    robot.limelight.updateRobotOrientation(
                            MathUtils.normalizeDegrees(
                                    robot.drive.getPose().getRotation().getDegrees()
                                            + Math.toDegrees(robot.turret.getPosition()) + 90.0, true)
                    );

                    Pose3D botPose = llResult.getBotpose_MT2();

                    if (botPose != null) {
                        Pose2d pinpointPose = robot.drive.getPose();
                        double x = DistanceUnit.INCH.fromMeters(botPose.getPosition().y);
                        double y = -DistanceUnit.INCH.fromMeters(botPose.getPosition().x);

                        if (x > -80 && x < 80 && y > -80 && y < 80) {
                            return new Pose2d(x, y, pinpointPose.getRotation());
                        }
                    }
                }
            }
        }

        return null;
    }

    private Pose2d getLimelightPoseMT1() {
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


    public double getMedianWallAngle() {
        if (robot.camera.medianWallAngle.isEmpty()) {
            return Double.NaN;
        }

        ArrayList<Double> sortedMedianWallAngle = new ArrayList<>(robot.camera.medianWallAngle);
        Collections.sort(sortedMedianWallAngle);

        if (sortedMedianWallAngle.size() % 2 == 1) {
            return sortedMedianWallAngle.get(sortedMedianWallAngle.size() / 2);
        }

        return (
            (sortedMedianWallAngle.get(sortedMedianWallAngle.size() / 2 - 1) +
             sortedMedianWallAngle.get(sortedMedianWallAngle.size() / 2)) / 2.0
        );
    }

    public void updateMedianReadings(Pose2d llPose) {
        medianWallAngle.add(robot.turret.angleToWall(llPose));
    }
}

