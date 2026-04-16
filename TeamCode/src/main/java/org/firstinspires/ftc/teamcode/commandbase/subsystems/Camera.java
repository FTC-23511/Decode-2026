package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.posesToAngle;
import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import android.util.Log;
import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Transform2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.MathUtils;
import com.seattlesolvers.solverslib.util.TelemetryEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.vision.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.vision.RectProcessor;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.MathFunctions;
import org.firstinspires.ftc.teamcode.globals.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import java.util.TreeMap;
import java.util.concurrent.TimeUnit;

@Config
public class Camera extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    public boolean enabled = false;
    public ArrayList<AprilTagDetection> detections = null;
    public AprilTagProcessor aprilTagProcessor;
    public RectProcessor rectProcessor;
    public VisionPortal visionPortal;
    public Transform2d relocalizationChange;

    public double cameraY = -1;
    public double cameraH = -1;

    private boolean recordReadings = false;

    public void setRecordReadings(boolean recordReadings) {
        this.recordReadings = recordReadings;
    }

    public boolean isRecordReadingsEnabled() {
        return recordReadings;
    }

    private final ArrayList<Pose2d> cameraPoseEstimates = new ArrayList<>();

    public final InterpLUT roiYOffsetLUT = new InterpLUT(
            Arrays.asList(0.0,  28.0,   39.0,  56.0, 67.0,  80.616, 99.31,  112.00, 120.0, 136.0, 150.0), // input: distance between robot and AprilTag (inches)
            Arrays.asList(240.0, 230.0, 129.8, 81.0, 67.0,  45.0,   33.2,   25.0,   22.0,  20.0,   20.0) // output: camera region of interest Y Offset
    );

    private final TreeMap<Double, Pose2d> pinpointHistory = new TreeMap<>();
    private static final double HISTORY_LIFESPAN_SECONDS = 1.5;

    public static double KALMAN_GAIN_POS = 0.1;
    public static double KALMAN_GAIN_HEADING = 0.1;

    private double lastCorrectionX = 0, lastCorrectionY = 0, lastCorrectionHeading = 0;
    private double lastStaleness = 0;

    // For Testing
    public Camera() {

    }

    public Camera(HardwareMap hwMap) {
        enabled = true;
        roiYOffsetLUT.createLUT();
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

        rectProcessor = new RectProcessor()
                .setRoi(new Rect(0, 0, 640, 480));

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTagProcessor)
                .addProcessor(rectProcessor);

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

    public void init() {
        if (!TESTING_OP_MODE) {
            if (OP_MODE_TYPE.equals(Constants.OpModeType.TELEOP)) {
                updateROI(END_POSE);
            } else {
                updateROI(robot.drive.getPose());
            }
        }
    }

    public void addPose(double timestamp, Pose2d pose) {
        pinpointHistory.put(timestamp, pose);
        while (!pinpointHistory.isEmpty() && pinpointHistory.firstKey() < timestamp - HISTORY_LIFESPAN_SECONDS) {
            pinpointHistory.pollFirstEntry();
        }
    }

    public void updateFilter(double cameraTimestamp, Pose2d cameraPose) {
        updateFilter(cameraTimestamp, cameraPose, KALMAN_GAIN_POS, KALMAN_GAIN_HEADING);
    }

    public void updateFilter(double cameraTimestamp, Pose2d cameraPose, double gainPos, double gainHeading) {
        Map.Entry<Double, Pose2d> historicalPinpointEntry = pinpointHistory.floorEntry(cameraTimestamp);

        if (historicalPinpointEntry == null) {
            return;
        }

        Pose2d historicalPinpointPose = historicalPinpointEntry.getValue();

        double translationErrorX = cameraPose.getX() - historicalPinpointPose.getX();
        double translationErrorY = cameraPose.getY() - historicalPinpointPose.getY();
        double headingError = MathUtils.normalizeRadians(cameraPose.getHeading() - historicalPinpointPose.getHeading(), false);

        double correctionX = translationErrorX * gainPos;
        double correctionY = translationErrorY * gainPos;
        double correctionHeading = headingError * gainHeading;

        lastCorrectionX = correctionX;
        lastCorrectionY = correctionY;
        lastCorrectionHeading = correctionHeading;

        Pose2d currentPose = robot.drive.getPose();
        Pose2d correctedPose = new Pose2d(
                currentPose.getX() + correctionX,
                currentPose.getY() + correctionY,
                MathUtils.normalizeRadians(currentPose.getHeading() + correctionHeading, false)
        );


        robot.drive.setPose(correctedPose);
        relocalizationChange = currentPose.minus(correctedPose);

        Log.d("Kalman", "Correction X: " + correctionX + " Y: " + correctionY);
    }

    public void undoRelocalization() {
        robot.drive.setPose(robot.drive.getPose().plus(relocalizationChange));
    }

    public void initHasMovement() {
        if (!Constants.TESTING_OP_MODE) {
            visionPortal.stopLiveView();
        }
    }

    /**
     * Gets the exact timestamp of the best AprilTag detection in seconds
     * and updates the latency/staleness variable.
     * @return the timestamp in seconds, or -1 if no detection is found.
     */
    public double getDetectionTimestamp() {
        if (detections != null && !detections.isEmpty()) {
            AprilTagDetection detection = cleanDetection(detections);
            if (detection != null && detection.robotPose != null) {
                // Update latency/staleness right here instead of doing it later
                lastStaleness = (System.nanoTime() - detection.frameAcquisitionNanoTime) / 1000000.0;
                return detection.frameAcquisitionNanoTime / 1e9;
            }
        }
        return -1;
    }

    /**
     * Updates internal camera result
     * @param n max number of times to attempt reading to get a valid result
     */
    public void updateArducam(int n) {
        if (enabled) {
            detections = null;

            if (TESTING_OP_MODE) {
                updateDecimation(TEST_DISTANCE);
            } else {
                updateDecimation(GOAL_POSE().minus(robot.drive.getPose()).getTranslation().getNorm());
            }

            updateROI(robot.drive.getPose());

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
    }

    /**
     * Relocalizes the robot using Arducam detections with a 100% hard reset.
     * @return true if relocalization was successful, false otherwise.
     */
    public boolean relocalizeArducam() {
        if (detections != null && !detections.isEmpty()) {
            Pose2d cameraPose = getCameraPose();
            double timestamp = getDetectionTimestamp();
            if (cameraPose != null && timestamp != -1) {
                Pose2d avgPose = getAverageCameraPose(cameraPose);

                // Hard reset (1.0 gain) as requested
                updateFilter(timestamp, avgPose, 1.0, 1.0);

                return true;
            }
        }

        return false;
    }

    /**
     * Updates Arducam detections and then relocalizes the robot.
     * @param n max number of times to attempt reading to get a valid result
     * @return true if relocalization was successful, false otherwise.
     */
    public boolean relocalizeArducam(int n) {
        updateArducam(n);
        return relocalizeArducam();
    }

    public void updateROI(Pose2d robotPose) {
        if (robotPose == null) return;
        double distance;
        if (TESTING_OP_MODE) {
            distance = TEST_DISTANCE;
        }
        else {
            distance = APRILTAG_POSE().minus(robot.drive.getPose()).getTranslation().getNorm();
        }

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

    public double getTagID() {
        if (detections != null && !detections.isEmpty()) {
            AprilTagDetection detection = cleanDetection(detections);

            if (detection != null) {
                return detection.id;
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

    public void setCameraExposure(long exposureMs) {
        if (visionPortal != null && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure(exposureMs, TimeUnit.MILLISECONDS);
        }
    }

    public void setCameraGain(int gain) {
        if (visionPortal != null && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
        }
    }

    public void closeCamera() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    @Override
    public void periodic() {
        if (TESTING_OP_MODE) {
            return;
        }

        if (recordReadings) {
            updateArducam(1);

            if (detections != null && !detections.isEmpty()) {
                Pose2d cameraPose = getCameraPose();
                double timestamp = getDetectionTimestamp();

                if (cameraPose != null && timestamp != -1) {
                    // 1. Log the pose to history
                    addPose(timestamp, cameraPose);

                    // 2. THE AUTO-MAGIC FILTER: Decide if the data is trustworthy
                    double distanceToTag = Constants.APRILTAG_POSE().minus(robot.drive.getPose()).getTranslation().getNorm();

                    // We only trust the camera if we are within 72 inches of the tag,
                    // and the frame isn't excessively stale (e.g., under 50ms latency)
                    if (distanceToTag < 72.0 && lastStaleness < 50.0) {

                        // 3. Apply the "Nudge". Notice we use your low-gain Kalman
                        // variables (0.1), NOT a 1.0 hard reset!
                        updateFilter(timestamp, cameraPose, KALMAN_GAIN_POS, KALMAN_GAIN_HEADING);
                    }
                }
            }
        }
    }

    public void updateCameraTelemetry(TelemetryEx telemetryEx) {
        telemetryEx.addData("Camera: Last Staleness", lastStaleness);
        telemetryEx.addData("Camera: Last Correction X", lastCorrectionX);
        telemetryEx.addData("Camera: Last Correction Y", lastCorrectionY);
        telemetryEx.addData("Camera: Last Correction H", lastCorrectionHeading);
    }
}