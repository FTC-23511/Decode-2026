package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
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

    public static boolean keepIntakeOn = false;

    public final ElapsedTime currentTimer;
    public final ElapsedTime distanceTimer;
    public boolean withinDistance = false;
    public boolean withinCurrent = false;
    public boolean currentBuffered = false;

    public static MotorState motorState = MotorState.STOP;
    public static DistanceState distanceState = DistanceState.FOV_15;

    public Intake() {
        currentTimer = new ElapsedTime();
        distanceTimer = new ElapsedTime();
        currentTimer.reset();
        distanceTimer.reset();
    }

    public void init() {
        if (!TESTING_OP_MODE) {

        }
    }

    public void setIntake(MotorState motorState) {
        switch (motorState) {
            case STOP:
                robot.intakeMotor.set(0);
                break;
            case TRANSFER:
                robot.intakeMotor.set(INTAKE_TRANSFER_SPEED);
                break;
            case FORWARD:
                currentTimer.reset();
                currentBuffered = false;
                robot.intakeMotor.set(INTAKE_FORWARD_SPEED);
                robot.launcher.setTransfer(true);
                break;
            case REVERSE:
                robot.intakeMotor.set(INTAKE_REVERSE_SPEED);
                break;
        }

        Intake.motorState = motorState;
    }

    public void toggleIntakeMotor() {
        setIntake(motorState.equals(MotorState.FORWARD) ? MotorState.STOP : MotorState.FORWARD);
    }

    public void update() {
        robot.profiler.start("Intake Update");

        switch (motorState) {
            case FORWARD:
                updateDistanceSensor();

                if (currentBuffered) {
                    updateCurrentSensor();
                } else if (currentTimer.milliseconds() > INTAKE_CURRENT_BUFFER_TIME) {
                    currentTimer.reset();
                    currentBuffered = true;
                } else {
                    withinCurrent = false;
                }
                break;
            case TRANSFER:
                break;
            case REVERSE:
                break;
            case STOP:
                // No point of setting intakeMotor to 0 again
                break;
        }

        robot.profiler.end("Intake Update");
    }

    public double getDistance() {
        double distance;
        double voltage = robot.distanceSensor.getVoltage();

        if (Double.isNaN(voltage) || voltage == 0.0) {
            return -1;
        }

        switch (distanceState) {
            case FOV_20:
                distance = (voltage * 48.7) - 4.9;
                break;
            case FOV_27:
                distance = (voltage * 78.1) - 10.2;
                break;
            case FOV_15:
            default:
                distance = (voltage * 32.5) - 2.6;
                break;
        }

        return distance;
    }

    public void updateDistanceSensor() {
        double distance = getDistance();
        withinDistance = distance > 0 && distance <= INTAKE_DISTANCE_THRESHOLD;

        if (!withinDistance) {
            distanceTimer.reset();
        }
    }

    public void updateCurrentSensor() {
        withinCurrent = (((MotorEx) robot.intakeMotor.getMotor()).isOverCurrent());

        if (!withinCurrent) {
            currentTimer.reset();
        }
    }

    public boolean transferFull() {
        return (withinCurrent && currentTimer.milliseconds() >= INTAKE_CURRENT_TIME)
            && (withinDistance && distanceTimer.milliseconds() >= INTAKE_DISTANCE_TIME);
    }

    @Override
    public void periodic() {
        update();
    }
}
