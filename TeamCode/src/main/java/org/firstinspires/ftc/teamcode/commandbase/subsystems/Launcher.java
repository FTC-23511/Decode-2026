package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

public class Launcher extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public enum Motif {
        NOT_FOUND,
        GPP,
        PGP,
        PPG
    }

    private final PIDFController flywheelController = new PIDFController(FLYWHEEL_PIDF_COEFFICIENTS);
    public static Motif motifState = Motif.NOT_FOUND;
    public static boolean motifStateSet = false;
    private boolean activeControl = false;


    public Launcher() {
        flywheelController.setTolerance(FLYWHEEL_VEL_TOLERANCE);
    }

    public void init() {
        setRamp(OP_MODE_TYPE == OpModeType.AUTO);
        setHood(MIN_HOOD_ANGLE);
        setFlywheel(0, false);
    }

    public void setFlywheel(double vel, boolean setActiveControl) {
        flywheelController.setSetPoint(vel * M_S_TO_TICKS);
        activeControl = setActiveControl;
    }

    public void setActiveControl(boolean state) {
        activeControl = state;
    }

    public double getFlywheelTarget() {
        return flywheelController.getSetPoint();
    }

    private void updateFlywheel() {
        robot.profiler.start("Launcher Update");
        if (activeControl) {
            flywheelController.setF(FLYWHEEL_PIDF_COEFFICIENTS.f / (robot.getVoltage() / 12));
            robot.launchMotors.set(
                    flywheelController.calculate(robot.launchEncoder.getCorrectedVelocity())
            );
        } else {
            if (getFlywheelTarget() == 0) {
                robot.launchMotors.set(0);
            } else {
                robot.launchMotors.set(LAUNCHER_DEFAULT_ON_SPEED);
            }
        }
        robot.profiler.end("Launcher Update");
    }

    public void setRamp(boolean engaged) {
        robot.rampServo.set(engaged ? RAMP_ENGAGED : RAMP_DISENGAGED);
    }

    public void setHood(double angle) {
        // Solved from proportion (targetServo - minServo) / servoRange = (targetAngle - minAngle) / angleRange
        robot.hoodServo.set(
                (angle - MIN_HOOD_ANGLE) / (MAX_HOOD_ANGLE - MIN_HOOD_ANGLE) * (MAX_HOOD_SERVO_POS - MIN_HOOD_SERVO_POS) + MIN_HOOD_SERVO_POS
        );
    }

    public boolean flywheelReady() {
        return activeControl && flywheelController.atSetPoint();
    }

    private double targetHoodAngle = 0.0;
    private double targetFlywheelVelocity = 0.0;

    public void distanceToLauncher(double distance) {
        double heightDifference = TARGET_HEIGHT - SHOOTER_HEIGHT;

        double term = distance * distance + heightDifference * heightDifference;
        double optimalAngleRad = Math.atan2(
                distance * distance + heightDifference * Math.sqrt(term),
                distance * Math.sqrt(term)
        );

        double optimalAngleDeg = Math.toDegrees(optimalAngleRad);
        targetHoodAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, optimalAngleDeg));

        double angleRad = Math.toRadians(targetHoodAngle);
        double cosAngle = Math.cos(angleRad);
        double tanAngle = Math.tan(angleRad);

        double denominator = 2 * cosAngle * cosAngle * (distance * tanAngle - heightDifference);

        if (denominator > 0) {
            double velocitySquared = (GRAVITY * distance * distance) / denominator;
            targetFlywheelVelocity = Math.sqrt(Math.max(0, velocitySquared));
        } else {
            // Fallback: target is unreachable with current constraints
            targetFlywheelVelocity = 0;


        }
    }

    public double getTargetHoodAngle(){
        return targetHoodAngle;
    }

    public double getTargetFlywheelVelocity(){
        return targetFlywheelVelocity;
    }

    public LLStatus getLimelightStatus() {
        return robot.limelight.getStatus();
    }

    public double[] getTargetDegrees() {
        double[] targetDegrees = new double[2];
        LLResult result = robot.limelight.getLatestResult();

        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                int id = fiducial.getFiducialId();

                targetDegrees[0] = fiducial.getTargetYDegrees();
                targetDegrees[1] = fiducial.getTargetXDegrees();
            }
        }

        return targetDegrees;
    }

    public Pose2d getLimelightPose() {
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

//                        if (x > ) {

//                        }
                    }
                }
            }
        }

        return null;
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

    @Override
    public void periodic() {
        updateFlywheel();
    }
}
