package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

public class ClearLaunch extends CommandBase {
    private final Robot robot;
    private ElapsedTime timer;
    private boolean targetStateSolved = false;

    /**
     * Assumes {@link ReadyToLaunch} has already been performed
     * Command to clear out current balls inside the robot
     */
    public ClearLaunch() {
        robot = Robot.getInstance();
        addRequirements(robot.intake, robot.launcher, robot.turret);
        timer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        robot.turret.setActiveControl(true);
        robot.launcher.setActiveControl(true);
        robot.intake.setIntake(Intake.MotorState.TRANSFER);
        robot.intake.setPivot(Intake.PivotState.TRANSFER);
        timer.reset();
    }

    @Override
    public void end(boolean interrupted) {
        if (Constants.OP_MODE_TYPE.equals(Constants.OpModeType.TELEOP)) {
            robot.intake.setIntake(Intake.MotorState.STOP);
        }
        robot.turret.setActiveControl(false);
        robot.launcher.setActiveControl(false);
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > 2000; // TODO: replace with real end condition of the command
    }
}
