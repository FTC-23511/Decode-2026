package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.globals.MathFunctions;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.Arrays;
import java.util.List;

@Config
public class Launcher extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    private final PIDFController flywheelController = new PIDFController(FLYWHEEL_PIDF_COEFFICIENTS);

    private boolean activeControl = false;
    private double targetHoodAngle = MIN_HOOD_ANGLE;
    private double targetFlywheelVelocity = 0.0;
    private boolean impossible = true;
    public static double DISTANCE_OFFSET = -0.0; // -0.267

    private static final List<Double> launcherInput  = Arrays.asList(0.0, 4.29,   4.49,   4.76,   5.22,   5.65,   6.06,   6.47,   6.80,   7.53,   7.84,   9.0); // input: velocity (m/s)
    private static final List<Double> launcherOutput = Arrays.asList(0.0, 1040.0, 1100.0, 1180.0, 1380.0, 1467.0, 1567.0, 1700.0, 1767.0, 2000.0, 2200.0, 2500.0); // output: ticks/s

    private static final List<Double> launcherDistance = Arrays.asList(0.0,  1.5,      2.0,   2.5,  3.0,  3.5,  4.0,   4.5,   5.0); // distance from ball leaving robot to when it touches goal for first time (meters)
    private static final List<Double> shootingTime     = Arrays.asList(0.67, 0.58375,  0.5,   0.52, 0.70, 0.77, 0.80d, 0.83d, 0.86d); // time it takes for ball to leave robot to start of goal (seconds)

    public static final InterpLUT launcherLUT = new InterpLUT(
            launcherInput,
            launcherOutput
    );

    public static final InterpLUT inverseLauncherLUT = new InterpLUT(
            launcherOutput,
            launcherInput
    );

    public static final InterpLUT timeOfFlightLUT = new InterpLUT(
            launcherDistance,
            shootingTime
    );

    public Launcher() {
        launcherLUT.createLUT();
        inverseLauncherLUT.createLUT();
        timeOfFlightLUT.createLUT();
        flywheelController.setTolerance(FLYWHEEL_VEL_TOLERANCE);
    }

    public void init() {
        if (!TESTING_OP_MODE) {
            setRamp(OP_MODE_TYPE.equals(OpModeType.AUTO));
            setHood(MIN_HOOD_ANGLE);
        }

        setFlywheel(0, false);
        setTransfer(false);
    }

    public void setFlywheel(double targetVel, boolean setActiveControl) {
        flywheelController.setSetPoint(Range.clip(launcherLUT.get(targetVel), 0, FLYWHEEL_MAX_VELOCITY));
        targetFlywheelVelocity = flywheelController.getSetPoint();
        setActiveControl(setActiveControl);
    }

    /**
     * Method to turn the transfer motor on or off
     */
    public void setTransfer(boolean active) {
        robot.transferMotor.set(active ? 1 : 0);
    }

    public void setLauncher(Pose2d robotPose) {
        double[] errorsAngleVelocity = MathFunctions.distanceToLauncherValues(
                MathFunctions.VirtualGoalSolver.solve(
                        robotPose,
                        new Vector2d(),
                        0,
                        GOAL_POSE(),
                        timeOfFlightLUT
                ).effectiveDistance
        );
        setFlywheel(errorsAngleVelocity[0], true);
        setHood(errorsAngleVelocity[1]);
    }

    /**
     * Used for INTERNAL TESTING OPMODES ONLY
     * @param vel velocity in ticks/second
     */
    public void setFlywheelTicks(double vel) {
        flywheelController.setSetPoint(vel);
        setActiveControl(true);
    }

    public double getTargetHoodAngle() {
        return targetHoodAngle;
    }

    public double getTargetFlywheelVelocity() {
        return targetFlywheelVelocity;
    }

    public void setActiveControl(boolean state) {
        activeControl = state;
        if (!state) {
            setHood(targetHoodAngle);
        }
    }

    public boolean getActiveControl() {
        return activeControl;
    }

    public double getFlywheelTarget() {
        return flywheelController.getSetPoint();
    }

    private void update() {
        robot.profiler.start("Launcher Update");
        flywheelController.setCoefficients(FLYWHEEL_PIDF_COEFFICIENTS);

        if (Drive.robotInZone(robot.drive.getPose()) && ENABLE_ZONE_CONTROL) {
            double[] targetLauncherValues = MathFunctions.distanceToLauncherValues(robot.getShotSolution().effectiveDistance);

            if (Double.isNaN(targetLauncherValues[0])) {
                impossible = true;
            } else {
                setFlywheel(targetLauncherValues[0], true);
                setHood(targetLauncherValues[1]);
            }
        }

        if (activeControl) {
            double flywheelVel = robot.launchEncoder.getCorrectedVelocity();
//            double transferVel = robot.transferEncoder.getCorrectedVelocity();

//            robot.launcher.flywheelController.setF(FLYWHEEL_PIDF_COEFFICIENTS.f * (robot.getVoltage() / DEFAULT_VOLTAGE));
//            robot.launcher.transferController.setF(TRANSFER_PIDF_COEFFICIENTS.f * (robot.getVoltage() / DEFAULT_VOLTAGE));

            robot.launchMotors.set(flywheelController.calculate(flywheelVel));
//            robot.transferMotor.set(transferController.calculate(transferVel));

            if (flywheelController.atSetPoint()) {
                impossible = false;
                setHood(targetHoodAngle);
            } else if (!TESTING_OP_MODE) {
                // hood compensation
                double adjustedHoodAngle = MathFunctions.getHoodAngleFromVelocity(
                        robot.getShotSolution().effectiveDistance,
                        inverseLauncherLUT.get(flywheelVel)
                );
                if (Double.isNaN(adjustedHoodAngle)) {
                    impossible = true;
                } else {
                    impossible = false;
                    setHood(adjustedHoodAngle, true);
                }
            }
        } else {
            if (getFlywheelTarget() == 0) {
                robot.launchMotors.set(0);
//                robot.transferMotor.set(0);
            } else {
                robot.launchMotors.set(LAUNCHER_DEFAULT_ON_SPEED);
//                robot.transferMotor.set(TRANSFER_DEFAULT_ON_SPEED);
            }

            impossible = true;
        }
        robot.profiler.end("Launcher Update");
    }

    public void setRamp(boolean engaged) {
        robot.rampServo.set(engaged ? RAMP_ENGAGED : RAMP_DISENGAGED);
    }

    public void setHood(double angle) {
        setHood(angle, false);
    }

    private void setHood(double angle, boolean compensation) {
        if (!compensation) {
            targetHoodAngle = angle;
        }

        robot.hoodServo.set(
                (angle - MIN_HOOD_ANGLE) / (MAX_HOOD_ANGLE - MIN_HOOD_ANGLE) * (MAX_HOOD_SERVO_POS - MIN_HOOD_SERVO_POS) + MIN_HOOD_SERVO_POS
        );
    }

    /**
     * Use launchValid() instead as it accounts for hood compensation
     * @return if the flywheel is at target
     */
    @Deprecated
    public boolean flywheelReady() {
        return activeControl && flywheelController.atSetPoint();
    }

    public boolean launchValid() {
        return activeControl && !impossible;
    }

    public boolean launchWillBeValid() {
        if (!activeControl) return false;

        double currentVel = robot.launchEncoder.getCorrectedVelocity();
        double targetVel = flywheelController.getSetPoint();
        double maxDeltaVel = FLYWHEEL_ACCEL * BALL_TRANSFER_TIME;
        
        double predictedVel;
        if (targetVel > currentVel) {
            predictedVel = Math.min(targetVel, currentVel + maxDeltaVel);
        } else {
            predictedVel = Math.max(targetVel, currentVel - maxDeltaVel);
        }

        if (Math.abs(targetVel - predictedVel) <= FLYWHEEL_VEL_TOLERANCE) {
            return true;
        } else if (!TESTING_OP_MODE) {
            double adjustedHoodAngle = MathFunctions.getHoodAngleFromVelocity(
                    robot.getShotSolution().effectiveDistance,
                    inverseLauncherLUT.get(predictedVel)
            );
            return !Double.isNaN(adjustedHoodAngle);
        }
        
        return false;
    }

    @Override
    public void periodic() {
        update();
    }
}