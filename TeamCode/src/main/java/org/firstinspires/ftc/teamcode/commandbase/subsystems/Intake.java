package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.globals.Robot;

public class Intake extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public enum MotorState {
        REVERSE,
        STOP,
        FORWARD,
        TRANSFER
    }

    // Used for normal distance mode (AnalogInput) on distance sensor
    public enum DistanceState {
        FOV_15,
        FOV_20,
        FOV_27
    }

    public boolean intakeJammed = false;
    private final ElapsedTime intakeTimer;
    public final ElapsedTime distanceTimer;
    public boolean withinDistance = false;
    public static MotorState motorState = MotorState.STOP;
    public static DistanceState distanceState = DistanceState.FOV_15;

    public Intake() {
        intakeTimer = new ElapsedTime();
        distanceTimer = new ElapsedTime();
        intakeTimer.reset();
        distanceTimer.reset();
    }

    public void init() {
        if (!TESTING_OP_MODE) {

        }
    }

    public void setIntake(MotorState motorState) {
        switch (motorState) {
            case STOP:
                robot.intakeMotors.set(0);
                break;
            case TRANSFER:
                robot.intakeMotors.set(INTAKE_TRANSFER_SPEED);
                break;
            case FORWARD:
                robot.intakeMotors.set(INTAKE_FORWARD_SPEED);
                break;
            case REVERSE:
                robot.intakeMotors.set(INTAKE_REVERSE_SPEED);
                break;
        }

        Intake.motorState = motorState;
    }

    public void toggleIntakeMotor() {
        setIntake(motorState.equals(MotorState.FORWARD) ? MotorState.STOP : MotorState.FORWARD);
    }

    public void update() {
        robot.profiler.start("Intake Update");

        robot.profiler.start("Distance Sensor");
//        updateDistanceSensors();
        robot.profiler.end("Distance Sensor");

        switch (motorState) {
            case FORWARD:
//                if (transferFull()) {
//                    setIntake(MotorState.STOP);
//                }
//                intakeJammed = true;
//                intakeTimer.reset();
//                setIntake(MotorState.REVERSE);
//
//                if (((MotorEx) robot.intakeMotors.getMotor()).isOverCurrent()) {
//
//                }
                break;
            case TRANSFER:
                break;
            case REVERSE:
                if (intakeJammed && intakeTimer.milliseconds() >= INTAKE_UNJAM_TIME) {
                    setIntake(MotorState.FORWARD);
                    intakeJammed = false;
                    intakeTimer.reset();
                }
                break;
            case STOP:
                // No point of setting intakeMotor to 0 again
                break;
        }

        robot.profiler.end("Intake Update");
    }

    public double getDistance() {
        double distance;

        switch (distanceState) {
            case FOV_20:
                distance = (robot.distanceSensor.getVoltage() * 48.7) - 4.9;
                break;
            case FOV_27:
                distance = (robot.distanceSensor. getVoltage() * 78.1) - 10.2;
                break;
            case FOV_15:
            default:
                distance = (robot.distanceSensor.getVoltage() * 32.5) - 2.6;
                break;
        }

        return distance;
    }

    public void updateDistanceSensors() {
        if (motorState.equals(MotorState.FORWARD)) {
            withinDistance = INTAKE_DISTANCE_THRESHOLD >= getDistance();

            if (!withinDistance) {
                distanceTimer.reset();
            }
        } else {
            withinDistance = false;
        }
    }

    public boolean transferFull() {
        return withinDistance && distanceTimer.milliseconds() >= INTAKE_DISTANCE_TIME;
    }

    @Override
    public void periodic() {
        update();
    }
}
