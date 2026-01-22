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
import com.seattlesolvers.solverslib.util.MathUtils;

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
import org.firstinspires.ftc.teamcode.globals.MathFunctions;
import org.firstinspires.ftc.teamcode.globals.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.TimeUnit;

public class Camera {
    private final Robot robot = Robot.getInstance();
    public boolean enabled = false;
    public static Motif motifState = Motif.NOT_FOUND;
    public ArrayList<AprilTagDetection> detections = null;
    public AprilTagProcessor aprilTagProcessor;
    public RectProcessor rectProcessor;
    public VisionPortal visionPortal;

    public double cameraY = -1;
    public double cameraH = -1;

    private final ArrayList<Pose2d> cameraPoseEstimates = new ArrayList<>();

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

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTagProcessor);

        if (TESTING_OP_MODE) {
            rectProcessor = new RectProcessor();
            rectProcessor.setRoi(new Rect(0, 0, 640, 480));
            builder.addProcessor(rectProcessor);
        }

        visionPortal = builder.build();

        try {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure(15, TimeUnit.MILLISECONDS);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
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
     * Updates internal camera result
     * @param n max number of times to attempt reading to get a valid result
     */
    public void updateCameraResult(int n) {
        detections = null;
        updateROI(robot.drive.getPose());
        updateDecimation(GOAL_POSE().minus(robot.drive.getPose()).getTranslation().getNorm());
        for (int i = n; i > 0; i--) {
            detections = aprilTagProcessor.getDetections();

            if (detections != null && !detections.isEmpty()) {
                Pose2d cameraPose = getCameraPose();

                if (cameraPose != null) {
                    updateCameraPoseReadings(cameraPose);
                }

                break;
            }
        }
    }

    public void updateROI(Pose2d robotPose) {
        if (robotPose == null) return;

        double distance = APRILTAG_POSE().minus(robot.drive.getPose()).getTranslation().getNorm();

        int finalY = (int) roiYOffsetLUT.get(distance);
        int finalH = (int) MathFunctions.mapEquation(distance, 30.0, 240.0, 144.0, 96.0);

        // For logging
        cameraY = finalY;
        cameraH = finalH;

        Rect calculatedRoi = new Rect(0, finalY, 640, finalH);

        aprilTagProcessor.setRegionOfInterest(calculatedRoi);
        if (rectProcessor != null) {
            rectProcessor.setRoi(calculatedRoi);
        }
    }

    public void updateDecimation(double distance) {
        if (distance < DECIMATION_THRESHOLD && !USE_CLOSE_DECIMATION) {
            aprilTagProcessor.setDecimation(CAMERA_CLOSE_DECIMATION);
            USE_CLOSE_DECIMATION = true;
        } else if (distance > DECIMATION_THRESHOLD && USE_CLOSE_DECIMATION) {
            aprilTagProcessor.setDecimation(CAMERA_FAR_DECIMATION);
            USE_CLOSE_DECIMATION = false;
        }
    }

    public double txOffset(@NonNull Pose2d robotPose, Pose2d goalPose) {
        double angleToGoal = Math.toDegrees(posesToAngle(robotPose, goalPose));
        double angleToATag = Math.toDegrees(posesToAngle(robotPose, APRILTAG_POSE()));

        return MathUtils.normalizeDegrees(angleToATag - angleToGoal, false);
    }

    public double getTxOffset(Pose2d robotPose) {
        if (robotPose == null) {
            return 0;
        }

        double finalOffset = txOffset(robotPose, robot.turret.adjustedGoalPose());
        RobotLog.aa("final offset", String.valueOf(finalOffset));
        return finalOffset;
    }

    public double[] getTargetDegrees() {
        if (detections != null && !detections.isEmpty()) {
            AprilTagDetection detection = cleanDetection(detections);

            if (detection != null) {
                return MathFunctions.pixelsToDegrees(detection.center);
            }
        }

        return null;
    }


    public Pose2d getCameraPose() {
        if (detections != null && !detections.isEmpty()) {
            AprilTagDetection detection = cleanDetection(detections);

            if (detection != null) {
                Pose3D robotPose = detection.robotPose;

                if (robotPose != null) {
                    return new Pose2d(
                            robotPose.getPosition().y,
                            -robotPose.getPosition().x,
                            robot.drive.getPose().getHeading()
                    );
                }
            }
        }

        return null;
    }

    private void updateCameraPoseReadings(Pose2d cameraPose) {
        cameraPoseEstimates.add(cameraPose);

        if (cameraPoseEstimates.size() > 3) {
            cameraPoseEstimates.remove(0);
        }
    }

    public Pose2d getAverageCameraPose(Pose2d cameraPose) {
        double avgX = cameraPoseEstimates.stream()
                .mapToDouble(Pose2d::getX)
                .average()
                .orElse(cameraPose.getX());

        double avgY = cameraPoseEstimates.stream()
                .mapToDouble(Pose2d::getY)
                .average()
                .orElse(cameraPose.getY());

        double avgHeading = cameraPoseEstimates.stream()
                .mapToDouble(Pose2d::getHeading)
                .average()
                .orElse(cameraPose.getHeading());

        // Update robot variable with the averaged values
        return new Pose2d(avgX, avgY, avgHeading);
    }

    public double getTagHeight() {
        if (detections != null && !detections.isEmpty()) {
            AprilTagDetection detection = cleanDetection(detections);

            if (detection != null) {
                return MathFunctions.getPointDimensions(detection.corners)[1];
            }
        }

        return -1;
    }

    public void writeCameraTelemetry(Telemetry telemetry) {
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

    public void closeCamera() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}

