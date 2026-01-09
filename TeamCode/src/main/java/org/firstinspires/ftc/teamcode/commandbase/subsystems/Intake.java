package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;

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


    public boolean intakeJammed = false;
    private final ElapsedTime intakeTimer;
    public final ElapsedTime distanceTimer;
    public boolean withinDistance = false;
    public static MotorState motorState = MotorState.STOP;

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
            if (motorState.equals(MotorState.FORWARD)) {
                setIntake(MotorState.STOP);
            } else if (motorState.equals(MotorState.STOP)) {
                setIntake(MotorState.FORWARD);
            }
        }



    public static SequentialCommandGroup ActiveStopIntake() {
        return new SequentialCommandGroup(
                new SetIntake(MotorState.REVERSE),
                new WaitCommand(100),
                new SetIntake(MotorState.STOP)
        );
    }
}
