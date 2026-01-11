package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.globals.Robot;

/**
 * Intake Subsystem
 * Manages the collection and transfer of game pieces.
 * Features a safety interlock with the Launcher to prevent jams.
 */
public class Intake extends SubsystemBase {
    // Singleton hardware access
    private final Robot robot = Robot.getInstance();

    /**
     * MotorState defines the directional logic for the intake rollers.
     */
    public enum MotorState {
        REVERSE,  // Spits out game pieces
        STOP,     // Motors off
        FORWARD,  // Collecting from the floor
        TRANSFER  // Moving pieces from internal storage to the launcher
    }

    // Status flags and timers for automation/jam detection
    public boolean intakeJammed = false;
    private final ElapsedTime intakeTimer;
    public final ElapsedTime distanceTimer;
    public boolean withinDistance = false;

    // Tracks the current state of the motor globally for this subsystem
    public static MotorState motorState = MotorState.STOP;

    public Intake() {
        intakeTimer = new ElapsedTime();
        distanceTimer = new ElapsedTime();
        intakeTimer.reset();
        distanceTimer.reset();
    }

    public void init() {
        // Place default initialization logic here (e.g., servo home positions)
        if (!TESTING_OP_MODE) {
            // Logic for match start
        }
    }

    /**
     * Sets the intake motor power based on the desired state.
     * Includes a safety check to ensure the flywheel is ready before intaking.
     *
     * @param motorState The target state (FORWARD, REVERSE, STOP, TRANSFER)
     */
    public void setIntake(MotorState motorState) {
        // Minimum velocity (ticks/s) required for the launcher to be considered "running"
        final double MIN_LAUNCHER_VELOCITY = 300.0;

        // --- FLYWHEEL SAFETY INTERLOCK ---
        // If we are trying to intake (FORWARD) or feed the shooter (TRANSFER),
        // we check if the flywheel is spinning fast enough to handle the game piece.
        if ((motorState == MotorState.FORWARD || motorState == MotorState.TRANSFER)) {
            double currentFlywheelVel = Math.abs(robot.launcher.getFlywheelVelocity());

            // If flywheel is too slow, we force the intake to STOP to prevent a "log-jam"
            if (currentFlywheelVel < MIN_LAUNCHER_VELOCITY) {
                motorState = MotorState.STOP;
            }
        }

        // Apply power constants defined in Constants.java based on the final state
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
                // Reverse ignores the flywheel safety check so you can always clear jams
                robot.intakeMotors.set(INTAKE_REVERSE_SPEED);
                break;
        }

        Intake.motorState = motorState;
    }

    /**
     * Simple toggle logic used for driver "hotkeys" (e.g., a single button to start/stop).
     */
    public void toggleIntakeMotor() {
        if (motorState.equals(MotorState.FORWARD)) {
            setIntake(MotorState.STOP);
        } else if (motorState.equals(MotorState.STOP)) {
            setIntake(MotorState.FORWARD);
        }
    }

    /**
     * ActiveStopIntake Command
     * Instead of just cutting power, this briefly reverses the motors to "kick"
     * game pieces back slightly, ensuring they aren't rubbing against the rollers
     * when the intake is off.
     *
     * @return A command group to be scheduled by the CommandScheduler.
     */
    public static SequentialCommandGroup ActiveStopIntake() {
        return new SequentialCommandGroup(
                new SetIntake(MotorState.REVERSE), // Brief pulse backwards
                new WaitCommand(75),              // Duration of the pulse
                new SetIntake(MotorState.STOP)     // Final halt
        );
    }
}
