package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.posesToAngle;
import static org.firstinspires.ftc.teamcode.globals.Constants.ALLIANCE_COLOR;
import static org.firstinspires.ftc.teamcode.globals.Constants.APRILTAG_POSE;
import static org.firstinspires.ftc.teamcode.globals.Constants.GOAL_POSE;

import android.util.Size;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

public class Camera {
    private final Robot robot = Robot.getInstance();
    public boolean enabled = false;
    public static Motif motifState = Motif.NOT_FOUND;
    public ArrayList<AprilTagDetection> detections = null;
    public ArrayList<Double> medianWallAngle = new ArrayList<>();
    public AprilTagProcessor aprilTagProcessor;
    public VisionPortal visionPortal;

    public final InterpLUT cameraInterplut = new InterpLUT(
            Arrays.asList(-Math.PI/2, -0.94, -0.9, -Math.PI/4, -0.6, -0.5, -0.3, -0.1, 0.25), // input: angle formed by lines between robot to goal and far field wall
            Arrays.asList(-12.0,      -12.0,  0.0,  0.0,        1.67, 4.67, 7.41, 14.14, 14.14) // output: new goal pos (inches)
    );

    public enum Motif {
        NOT_FOUND,
        GPP,
        PGP,
        PPG
    }

    public Camera(HardwareMap hwMap) {
        cameraInterplut.createLUT();
        init(hwMap);
    }

    public void init(HardwareMap hwMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(false)
//                .setDrawAxes(true)
//                .setDrawTagOutline(true)
//                .setDrawCubeProjection(true)
                .setNumThreads(2)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setLensIntrinsics(549.651, 549.651, 317.108, 236.644) // 640x480: 549.651, 549.651, 317.108, 236.644; 320x240: 281.5573273, 281.366942, 156.3332591, 119.8965271
                .setCameraPose( // TODO: Fix offsets
                        new Position(DistanceUnit.MM, 0, 0, 0, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, 0, 64.506770, 180, 0))
                .build();

        aprilTagProcessor.setDecimation(2); // increases fps, but reduces range

        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTagProcessor)
                .build();
    }

    public void initHasMovement() {
        visionPortal.stopLiveView();
    }

    /**
     * Updates internal limelight result in the Turret class
     * @param n max number of times to attempt reading to get a valid result
     */
    public void updateCameraResult(int n) {
        detections = null;

        for (int i = n; i > 0; i--) {
            detections = aprilTagProcessor.getFreshDetections();

            if (detections != null && !detections.isEmpty()) {
                break;
            } else {
                detections = null;
            }
        }

    }

    public double txOffset(@NonNull Pose2d robotPose, Pose2d goalPose) {
        double angleToGoal = Math.toDegrees(posesToAngle(robotPose, goalPose));
        double angleToATag = Math.toDegrees(posesToAngle(robotPose, APRILTAG_POSE()));

        return angleToATag - angleToGoal;
    }

    public double getTxOffset(Pose2d robotPose) {
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

        double finalOffset = txOffset(robotPose, adjustedGoal);
        RobotLog.aa("final offset", String.valueOf(finalOffset));
        return finalOffset;
    }

    public double[] getTargetDegrees() {
        if (detections != null && !detections.isEmpty()) {
            Point point = cleanDetection(detections).center;

            return new double[]{point.x - 320, point.y - 240};
        }

        return null;
    }

    public Pose2d getCameraPose() {
        if (detections != null && !detections.isEmpty()) {
            AprilTagPoseFtc ftcPose = cleanDetection(detections).ftcPose;

            return new Pose2d(ftcPose.x, ftcPose.y, ftcPose.yaw);
        }

        return null;
    }

    public void getCameraTelemetry(Telemetry telemetry) {
        if (detections != null && !detections.isEmpty()) {
            AprilTagDetection detection = cleanDetection(detections);

            if (detection != null && detection.metadata != null) {
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                    telemetry.addData("center", detection.center);
                }
            } else {
                telemetry.addLine("\n==== Tag not found");
            }
        }
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
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}

