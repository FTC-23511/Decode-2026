package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.globals.Robot;

public class Intake extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public enum MotorState {
        REVERSE,
        STOP,
        FORWARD,
        TRANSFER
    }

    public enum PivotState {
        FORWARD,
        TRANSFER,
        HOLD
    }

    // Used for normal distance mode on distance sensor
    public enum DistanceState {
        FOV_15,
        FOV_20,
        FOV_27
    }

    public boolean intakeJammed = false;
    private final ElapsedTime intakeTimer;
    private final ElapsedTime distanceTimer;
    private boolean withinDistance = false;
    public static MotorState motorState = MotorState.STOP;
    public static PivotState pivotState = PivotState.HOLD;
    public static DistanceState distanceState = DistanceState.FOV_15;

    public Intake() {
        intakeTimer = new ElapsedTime();
        distanceTimer = new ElapsedTime();
        intakeTimer.reset();
        distanceTimer.reset();
    }

    public void init() {
        if (!TESTING_OP_MODE) {
            if (OP_MODE_TYPE == OpModeType.AUTO) {
                setPivot(PivotState.HOLD);
            } else {
                setPivot(PivotState.FORWARD);
            }
        }
    }

    public void setPivot(PivotState pivotState) {
        switch (pivotState) {
            case HOLD:
                robot.intakePivotServo.set(INTAKE_PIVOT_HOLD);
                break;
            case TRANSFER:
                robot.intakePivotServo.set(INTAKE_PIVOT_TRANSFER);
                break;
            case FORWARD:
                robot.intakePivotServo.set(INTAKE_PIVOT_FORWARD);
                break;
        }

        Intake.pivotState = pivotState;
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

    public void toggleIntake() {
        if (pivotState.equals(PivotState.FORWARD)) {
            if (motorState.equals(MotorState.FORWARD)) {
                setIntake(MotorState.STOP);
            } else if (motorState.equals(MotorState.STOP)) {
                setIntake(MotorState.FORWARD);
            }
        }
    }

    public void updateIntake() {
        robot.profiler.start("Intake Update");

        if (getDistance() < DISTANCE_THRESHOLD) {
            withinDistance = true;
        } else {
            withinDistance = false;
            distanceTimer.reset();
        }

        switch (motorState) {
            case FORWARD:
//                if (transferFull()) {
//                    setPivot(PivotState.HOLD);
//                    setIntake(MotorState.STOP);
//                }

//                if (((MotorEx) robot.intakeMotors.getMotor()).isOverCurrent()) {
//                    intakeJammed = true;
//                    intakeTimer.reset();
//                    setIntake(MotorState.REVERSE);
//                }
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

    public void updateSensors() {
//        robot.frontDistanceSensor.update();
//        robot.backDistanceSensor.update();
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

    public boolean transferFull() {
        return withinDistance && distanceTimer.milliseconds() >= DISTANCE_TIME;

        //  return robot.frontDistanceSensor.isActive() && robot.frontDistanceSensor.isActive()
        //  && !intakeJammed;
    }

    @Override
    public void periodic() {
//        updateDistanceSensors();
        updateIntake();
    }

    public static SequentialCommandGroup ActiveStopIntake() {
        return new SequentialCommandGroup(
                new SetIntake(MotorState.FORWARD, PivotState.HOLD),
                new WaitCommand(250),
                new SetIntake(MotorState.STOP, PivotState.HOLD)
        );
    }
}
