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
        // Define the minimum velocity required for the intake to run
        // Adjust this value based on your flywheel's typical operating speed
        final double MIN_LAUNCHER_VELOCITY = 700.0;

        // Safety Check: If trying to move game pieces forward/transfer
        // but the flywheel is too slow, force the intake to STOP.
        if ((motorState == MotorState.FORWARD || motorState == MotorState.TRANSFER)) {
            // Access the launcher velocity through the Robot singleton
            double currentFlywheelVel = Math.abs(robot.launcher.getFlywheelVelocity());

            if (currentFlywheelVel < MIN_LAUNCHER_VELOCITY) {
                motorState = MotorState.STOP;
            }
        }

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
                // We allow REVERSE even if the flywheel is stopped to clear jams
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
                new WaitCommand(50),
                new SetIntake(MotorState.STOP)
        );
    }
}
