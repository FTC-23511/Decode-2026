package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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

    public enum DistanceState {
        FOV_15,
        FOV_20,
        FOV_27
    }

    public boolean intakeJammed = false;
    private final ElapsedTime intakeTimer;
    public static MotorState motorState = MotorState.STOP;
    public static PivotState pivotState = PivotState.HOLD;
    public static DistanceState distanceState = DistanceState.FOV_15;

    public Intake() {
        intakeTimer = new ElapsedTime();
        intakeTimer.reset();
    }

    public void init() {
        if (OP_MODE_TYPE == OpModeType.AUTO) {
            setPivot(PivotState.HOLD);
        } else {
            setPivot(PivotState.FORWARD);
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
                robot.intakeMotor.set(0);
                break;
            case TRANSFER:
                robot.intakeMotor.set(INTAKE_TRANSFER_SPEED);
                break;
            case FORWARD:
                robot.intakeMotor.set(INTAKE_FORWARD_SPEED);
                break;
            case REVERSE:
                robot.intakeMotor.set(INTAKE_REVERSE_SPEED);
                break;
        }
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
        if (pivotState.equals(PivotState.FORWARD)) {
            switch (motorState) {
                case FORWARD:
                    if (transferFull()) {
                        setPivot(PivotState.HOLD);
                        setIntake(MotorState.STOP);
                    }

                    if (robot.intakeMotor.isOverCurrent()) {
                        intakeJammed = true;
                        intakeTimer.reset();
                        setIntake(MotorState.REVERSE);
                    }
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
        } else if (pivotState.equals(PivotState.HOLD)) {
            setIntake(MotorState.STOP);
        }
        robot.profiler.end("Intake Update");
    }

    public boolean transferFull() {
        // TODO: Fix logic
        return false;
//        return (getDistance(distanceState) < MAX_DISTANCE_THRESHOLD) && (getDistance(distanceState) > MIN_DISTANCE_THRESHOLD);
    }

    /*
    public double getDistance(DistanceState distanceState){
        double distance = 0;

        switch (distanceState) {
            case FOV_15:
                distance = robot.distanceSensor.getVoltage() * 32.05 - 2.6;
                break;
            case FOV_20:
                distance =  robot.distanceSensor.getVoltage() * 48.70 - 4.9;
                break;
            case FOV_27:
                distance =  robot.distanceSensor.getVoltage() * 78.1 - 10.2;
                break;
        }

        Intake.distanceState = distanceState;
        return distance;
    }
     */

    @Override
    public void periodic() {
        updateIntake();
    }
}
