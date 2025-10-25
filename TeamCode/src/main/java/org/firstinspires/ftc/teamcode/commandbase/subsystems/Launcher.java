package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.globals.Robot;

public class Launcher extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public enum MotorState {
        REVERSE,
        STOP,
        FORWARD,
        HOLD
    }

    public static MotorState motorState = MotorState.STOP;

    public void init() {

    }

    public void setLauncher(MotorState motorState) {
        if (motorState.equals(MotorState.HOLD)) {
            robot.intakeMotor.set(0);
        } else {
            switch (motorState) {
                case FORWARD:
                    robot.launchMotors.set(INTAKE_FORWARD_SPEED);
                    break;
                case REVERSE:
                    robot.launchMotors.set(INTAKE_REVERSE_SPEED);
                    break;
                case STOP:
                    robot.launchMotors.set(0);
                    break;
            }
        }
        Launcher.motorState = motorState;
    }

    //    public double autoAim(Pose2d limelightHeading){
//        double robotHeading = robot.drive.getPose().getHeading();
//        return 0.0;
//    }
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


    public void updateLauncher() {
        // TODO: Add this
    }

    @Override
    public void periodic() {
        updateLauncher();
    }
}
