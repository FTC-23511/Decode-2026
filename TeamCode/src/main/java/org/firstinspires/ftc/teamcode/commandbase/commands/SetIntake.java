package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.globals.Robot;

public class SetIntake extends CommandBase {
    private final Robot robot;
    private final Intake.MotorState motorState;
    private final Intake.PivotState pivotState;
    private boolean waitForArtifacts;

    private ElapsedTime timer;

    public SetIntake(Intake.MotorState motorState, Intake.PivotState pivotState) {
        this(motorState, pivotState, false);
    }

    public SetIntake(Intake.MotorState motorState, Intake.PivotState pivotState, boolean waitForArtifacts) {
        robot = Robot.getInstance();
        this.motorState = motorState;
        this.pivotState = pivotState;
        this.waitForArtifacts = waitForArtifacts;

        timer = new ElapsedTime();

        addRequirements(robot.intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        robot.intake.setPivot(pivotState);
        robot.intake.setIntake(motorState);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        if (waitForArtifacts && robot.intake.transferFull()) {
            return true;
        }

        return !waitForArtifacts && timer.milliseconds() > 200;
    }
}
