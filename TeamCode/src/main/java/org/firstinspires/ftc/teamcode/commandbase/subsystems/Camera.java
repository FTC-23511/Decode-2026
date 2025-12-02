package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.posesToAngle;
import static org.firstinspires.ftc.teamcode.globals.Constants.ALLIANCE_COLOR;
import static org.firstinspires.ftc.teamcode.globals.Constants.APRILTAG_POSE;
import static org.firstinspires.ftc.teamcode.globals.Constants.GOAL_POSE;
import static org.firstinspires.ftc.teamcode.globals.Constants.USE_LIMELIGHT_MT1;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

public class Camera {
    private final Robot robot = Robot.getInstance();
    public static Motif motifState = Motif.NOT_FOUND;
    public ArrayList<AprilTagDetection> detections = null;
    public ArrayList<Double> medianWallAngle = new ArrayList<>();

    double bs = (ALLIANCE_COLOR.equals(Constants.AllianceColor.BLUE) ? -12.67 : -5.1);
    public final InterpLUT cameraInterplut = new InterpLUT(
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
        cameraInterplut.createLUT();

    }

    /**
     * Updates internal limelight result in the Turret class
     * @param n max number of times to attempt reading to get a valid result
     */
    public void updateCameraResult(int n) {
        detections = null;

        for (int i = n; i > 0; i--) {
            detections = robot.aprilTagProcessor.getFreshDetections();

            if (detections != null && !detections.isEmpty()) {
                break;
            } else {
                detections = null;
            }
        }

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
        double adjustment = cameraInterplut.get(offset);
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

    public Pose2d getCameraPose() {
        if (detections != null && !detections.isEmpty()) {
            AprilTagPoseFtc ftcPose = cleanDetection(detections).ftcPose;

            return new Pose2d(ftcPose.x, ftcPose.y, ftcPose.yaw);
        }

        return null;
    }

    public AprilTagDetection cleanDetection(ArrayList<AprilTagDetection> aprilTagDetections) {
        if (aprilTagDetections != null && !aprilTagDetections.isEmpty()) {

            for (AprilTagDetection aprilTagDetection : aprilTagDetections) {
                double id = aprilTagDetection.id;

                if ((Constants.ALLIANCE_COLOR.equals(Constants.AllianceColor.BLUE) && id == 20)
                        || (Constants.ALLIANCE_COLOR.equals(Constants.AllianceColor.RED) && id == 24)) {
                    return aprilTagDetection;
                }
            }
        }

        return null;
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

    public void closeCamera() {
        if (robot.visionPortal != null) {
            robot.visionPortal.close();
        }
    }
}

