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
            robot.intakeMotor.set(INTAKE_HOLD_SPEED);
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

    public double autoAim(Pose2d limelightHeading){
        double robotHeading = robot.drive.getPose().getHeading();
        return 0.0;
    }

    public void updateLauncher() {
        // TODO: Add this
    }

    @Override
    public void periodic() {
        updateLauncher();
    }
}
