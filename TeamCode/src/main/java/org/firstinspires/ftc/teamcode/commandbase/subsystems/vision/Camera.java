package org.firstinspires.ftc.teamcode.commandbase.subsystems.vision;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.posesToAngle;
import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import android.util.Log;
import android.util.Size;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.opencv.core.Point;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.concurrent.TimeUnit;

public class Camera {
    private final Robot robot = Robot.getInstance();
    public boolean enabled = false;
    public static Motif motifState = Motif.NOT_FOUND;
    public ArrayList<AprilTagDetection> detections = null;
    public ArrayList<Double> medianWallAngle = new ArrayList<>();
    public AprilTagProcessor aprilTagProcessor;
    public RectProcessor rectProcessor;
    public VisionPortal visionPortal;

    public double cameraY = -1;
    public double cameraH = -1;

    public final InterpLUT goalAdjustmentLUT = new InterpLUT(
            Arrays.asList(-Math.PI/2, -0.94, -0.9, -Math.PI/4, -0.6, -0.5, -0.3, -0.1, 0.25), // input: angle formed by lines between robot to goal and far field wall
            Arrays.asList(-12.0,      -12.0,  0.0,  0.0,        1.67, 4.67, 7.41, 14.14, 14.14), // output: new goal pos (inches)
            true
    );

    public final InterpLUT roiYOffsetLUT = new InterpLUT(
            Arrays.asList(0.0,  28.0,   39.0,  56.0, 67.0,  80.616, 99.31,  112.00, 120.0, 136.0, 150.0), // input: distance between robot and AprilTag (inches)
            Arrays.asList(240.0, 230.0, 129.8, 81.0, 67.0,  45.0,   33.2,   25.0,   22.0,  20.0,   20.0), // output: camera region of interest Y Offset
            true
    );

    public enum Motif {
        NOT_FOUND,
        GPP,
        PGP,
        PPG
    }

    public Camera(HardwareMap hwMap) {
        goalAdjustmentLUT.createLUT();
        roiYOffsetLUT.createLUT();
        init(hwMap);
    }

    public void init(HardwareMap hwMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(false)
//                .setDrawAxes(true)
//                .setDrawTagOutline(true)
//                .setDrawCubeProjection(true)
                .setNumThreads(3)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setRegionOfInterest(new Rect(0, 0,640, 480))
                .setLensIntrinsics(549.651, 549.651, 317.108, 236.644) // 640x480: 549.651, 549.651, 317.108, 236.644; 320x240: 281.5573273, 281.366942, 156.3332591, 119.8965271
                .setCameraPose( // TODO: Fix offsets (forward, which I think is x)
                        new Position(DistanceUnit.MM, 0, 0, 0, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, 0, 64.506770, 180, 0))
                .build();

        aprilTagProcessor.setDecimation(CAMERA_CLOSE_DECIMATION); // increases fps, but reduces range

        rectProcessor = new RectProcessor();
        rectProcessor.setRoi(new Rect(0, 0, 640, 480));

        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTagProcessor)
                .addProcessor(rectProcessor)
                .build();
        try {
            ExposureControl exposureControl = robot.camera.visionPortal.getCameraControl(ExposureControl.class);
            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure(15, TimeUnit.MILLISECONDS);

            GainControl gainControl = robot.camera.visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(100);
        } catch (Exception e) {
            Log.wtf("WHAT A TERRIBLE FAILURE.", "Camera Exposure/Gain Control got fried \n" + e);
        }
    }

    public void initHasMovement() {
        if (!Constants.TESTING_OP_MODE) {
            visionPortal.stopLiveView();
        }
    }

    /**
     * Updates internal limelight result in the Turret class
     * @param n max number of times to attempt reading to get a valid result
     */
    public void updateCameraResult(int n) {
        detections = null;
        updateROI(robot.drive.getPose());
        for (int i = n; i > 0; i--) {
            detections = aprilTagProcessor.getDetections();

            if (detections != null && !detections.isEmpty()) {
                Pose2d pose = getCameraPose();

                if (pose != null) {
                    updateMedianReadings(pose);
                    robot.turret.updateTurretPoseReadings(pose);
                }

                if (medianWallAngle.size() > 10) {
                    medianWallAngle.remove(0);
                }

                break;
            }
        }
    }

    public void updateROI(Pose2d robotPose) {
        if (robotPose == null) return;

        // 1. Get Distance (Inches)
        double distance = APRILTAG_POSE().minus(robot.drive.getPose()).getTranslation().getNorm();

        // 2. Define Keyframes (Using Top-Left Coordinates)

        // CLOSE (30 inches) -> Bottom Half
        double distClose = 30.0;
        double yClose = 240.0;    // Starts at pixel 240 (Middle)
        double hClose = 240.0;    // Height is 240 (Middle to Bottom)

        // FAR (144 inches) -> Top Strip
        double distFar = 144.0;
        double yFar = 0.0;        // Starts at pixel 0 (Top Edge)
        double hFar = 96.0;       // Height is 96 (Top Strip)

        // 3. Calculate Linear Interpolation
//        int finalY = (int) mapEquation(distance, distClose, yClose, distFar, yFar);
        int finalY = (int) roiYOffsetLUT.get(distance);
        int finalH = (int) mapEquation(distance, distClose, hClose, distFar, hFar);

        cameraY = finalY;
        cameraH = finalH;

        // 4. Create the Rect (Standard 0-640 coords)
        // x = 0 (Left Edge)
        // width = 640 (Full Width)
        Rect calculatedRoi = new Rect(0, finalY, 640, finalH);

        // 5. Update BOTH processors
        aprilTagProcessor.setRegionOfInterest(calculatedRoi);
        rectProcessor.setRoi(calculatedRoi);
    }

    /**
     * Standard Linear Equation Solver (Point-Slope Form)
     * Calculates y based on x, given two known points (x1,y1) and (x2,y2).
     */
    private double mapEquation(double x, double x1, double y1, double x2, double y2) {
        // 1. Clamp input to prevent the box from flying off screen
        // If we are closer than the 'Close' point, just stay at the 'Close' settings.
        if (x <= x1) return y1;
        if (x >= x2) return y2;

        // 2. Calculate Slope (m)
        double m = (y2 - y1) / (x2 - x1);

        // 3. Calculate Result (Point-Slope formula: y - y1 = m(x - x1))
        return y1 + m * (x - x1);
    }

    public void updateDecimation(double distance) {
        if (distance < DECIMATION_THRESHOLD && !USE_CLOSE_DECIMATION) {
            aprilTagProcessor.setDecimation(CAMERA_CLOSE_DECIMATION);
            USE_CLOSE_DECIMATION = true;
        } else if (USE_CLOSE_DECIMATION) {
            aprilTagProcessor.setDecimation(CAMERA_FAR_DECIMATION);
            USE_CLOSE_DECIMATION = false;
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
        double adjustment = goalAdjustmentLUT.get(offset);
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
            AprilTagDetection detection = cleanDetection(detections);

            if (detection != null) {
                return pixelsToDegrees(detection.center);
            }
        }

        return null;
    }

    public static double[] pixelsToDegrees(Point center) {
        double px = center.x;
        double py = center.y;

        double xNorm = (px - 317.108) / 549.651;
        double yNorm = (py - 236.644) / 549.651;

        double horizontalAngle = Math.toDegrees(Math.atan(xNorm));
        double verticalAngle = Math.toDegrees(Math.atan(yNorm));

        return new double[]{horizontalAngle, verticalAngle};
    }

    public Pose2d getCameraPose() {
        if (detections != null && !detections.isEmpty()) {
            AprilTagDetection detection = cleanDetection(detections);

            if (detection != null) {
                Pose3D robotPose = detection.robotPose;

                if (robotPose != null) {
                    return new Pose2d(robotPose.getPosition().y,
                            -robotPose.getPosition().x,
                            robot.drive.getPose().getHeading()
                    );
                }
            }
        }

        return null;
    }

    public double getTagHeight() {
        if (detections != null && !detections.isEmpty()) {
            AprilTagDetection detection = cleanDetection(detections);

            if (detection != null) {
                Point[] corners = detection.corners;

                // Initialize min and max variables
                double minY = Double.MAX_VALUE;
                double maxY = Double.MIN_VALUE;
                double minX = Double.MAX_VALUE;
                double maxX = Double.MIN_VALUE;

                // Loop through all 4 corners to find the bounds
                for (Point point : corners) {
                    if (point.y < minY) minY = point.y;
                    if (point.y > maxY) maxY = point.y;
                    if (point.x < minX) minX = point.x;
                    if (point.x > maxX) maxX = point.x;
                }

                return maxY - minY;
            }
        }

        return -1;
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

    public void updateMedianReadings(Pose2d cameraPose) {
        medianWallAngle.add(robot.turret.angleToWall(cameraPose));
    }

    public void closeCamera() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}

