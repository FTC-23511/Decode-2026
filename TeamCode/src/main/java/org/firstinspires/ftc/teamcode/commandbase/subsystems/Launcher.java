package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.globals.MathFunctions;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.Arrays;
import java.util.List;

@Config
public class Launcher extends SubsystemBase {
    private final Robot robot = Robot.getInstance();


    private final PIDFController flywheelController = new PIDFController(FLYWHEEL_PIDF_COEFFICIENTS);
    private final PIDFController transferController = new PIDFController(TRANSFER_PIDF_COEFFICIENTS);

    private boolean activeControl = false;
    private double targetHoodAngle = MIN_HOOD_ANGLE;
    private double targetFlywheelVelocity = 0.0;
    private double targetTransferVelocity = 0.0;
    private boolean impossible = true;
    public static double DISTANCE_OFFSET = -0.0; // -0.267


    private final List<Double> launcherInput  = Arrays.asList(-0.01, 0.0, 4.29,   4.49,   4.76,   5.22,   5.65,   6.06,   6.44,   6.86,   7.2,    10.0); // input: velocity (m/s)
    private final List<Double> launcherOutput = Arrays.asList(-0.01, 0.0, 1040.0, 1100.0, 1180.0, 1320.0, 1480.0, 1620.0, 1840.0, 2020.0, 2060.0, 2100.0); // output: ticks/s

    private final List<Double> launcherDistance  = Arrays.asList(-0.01, 0.0); // distance from ball leaving robot to when it touches goal for first time
    private final List<Double> shootingTime  = Arrays.asList(-0.01, 0.0); // time it takes for ball to leave robot to start of goal

    private final InterpLUT launcherLUT = new InterpLUT(
            launcherInput,
            launcherOutput,
            true
    );

    private final InterpLUT inverseLauncherLUT = new InterpLUT(
            launcherOutput,
            launcherInput,
            true
    );

    private final InterpLUT timeOfFlightLUT = new InterpLUT(
            launcherDistance,
            shootingTime,
            true
    );
    public Launcher() {
        launcherLUT.createLUT();
        inverseLauncherLUT.createLUT();
        timeOfFlightLUT.createLUT();
        flywheelController.setTolerance(FLYWHEEL_VEL_TOLERANCE);
        transferController.setTolerance(TRANSFER_VEL_TOLERANCE);
    }

    public void init() {
        if (!TESTING_OP_MODE) {
            setRamp(OP_MODE_TYPE.equals(OpModeType.AUTO));
            setHood(MIN_HOOD_ANGLE);
        }

        setFlywheel(0, false);
        setTransfer(0);
    }

    public void setFlywheel(double targetVel, boolean setActiveControl) {
        flywheelController.setSetPoint(Range.clip(launcherLUT.get(targetVel), 0, TRANSFER_MAX_VELOCITY));
        targetFlywheelVelocity = flywheelController.getSetPoint();
        setActiveControl(setActiveControl);
    }

    public void setTransfer(double targetVel) {
        transferController.setSetPoint(Range.clip(targetVel, 0, TRANSFER_MAX_VELOCITY));
        targetTransferVelocity = transferController.getSetPoint();
    }

    public void setLauncher(Pose2d robotPose) {
        double[] errorsAngleVelocity = MathFunctions.distanceToLauncherValues(
                MathFunctions.VirtualGoalSolver.solve(
                        robotPose,
                        new Vector2d(),
                        0,
                        GOAL_POSE(),
                        timeOfFlightLUT // <--- Pass the table here
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

    public double getTargetTransferVelocity() {
        return targetTransferVelocity;
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
        transferController.setCoefficients(TRANSFER_PIDF_COEFFICIENTS);

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
            double transferVel = robot.transferEncoder.getCorrectedVelocity();

//            robot.launcher.flywheelController.setF(FLYWHEEL_PIDF_COEFFICIENTS.f * (robot.getVoltage() / DEFAULT_VOLTAGE));
//            robot.launcher.transferController.setF(TRANSFER_PIDF_COEFFICIENTS.f * (robot.getVoltage() / DEFAULT_VOLTAGE));

            robot.launchMotors.set(flywheelController.calculate(flywheelVel));
            robot.transferMotor.set(transferController.calculate(transferVel));

            if (flywheelController.atSetPoint()) {
                impossible = false;
                setHood(targetHoodAngle);
            } else {
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
                robot.transferMotor.set(0);
            } else {
                robot.launchMotors.set(LAUNCHER_DEFAULT_ON_SPEED);
                robot.transferMotor.set(TRANSFER_DEFAULT_ON_SPEED);
            }

            impossible = true;
        }
        robot.profiler.end("Launcher Update");
    }

    public void setRamp(boolean engaged) {
        robot.rampServo.set(engaged ? RAMP_ENGAGED : RAMP_DISENGAGED);
    }

//    public void setStopper(boolean engaged) {
//        robot.stopperServo.set(engaged ? STOPPER_ENGAGED_POS : STOPPER_DISENGAGED_POS);
//    }

    public InterpLUT getTimeOfFlightLUT() {
        return timeOfFlightLUT;
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

    // use launchValid() instead as it accounts for hood compensation
    @Deprecated
    public boolean flywheelReady() {
        return activeControl && flywheelController.atSetPoint() && transferController.atSetPoint();
    }

    public boolean launchValid() {
        return activeControl && !impossible;
    }

    @Override
    public void periodic() {
        update();
    }
}
