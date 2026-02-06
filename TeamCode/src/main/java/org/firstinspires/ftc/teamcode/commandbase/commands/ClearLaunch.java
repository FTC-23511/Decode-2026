package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

public class ClearLaunch extends CommandBase {
    private final Robot robot;
    private final ElapsedTime timer;
    private final boolean preciseShots;
    private final boolean timeBased;

    /**
     * Assumes {@link FullAim} has already been performed or the robot is already aimed
     * Command to clear out current balls inside the robot
     */
    public ClearLaunch() {
        this(false);
    }

    /**
     * Assumes {@link FullAim} has already been performed or the robot is already aimed
     * Command to clear out current balls inside the robot
     * @param preciseShots whether the feeder should wait until flywheel is at target velocity for precise shots or just rapid fire
     */
    public ClearLaunch(boolean preciseShots) {
        this(preciseShots, false);
    }

    public ClearLaunch(boolean preciseShots, boolean timeBased) {
        robot = Robot.getInstance();
        timer = new ElapsedTime();
        this.preciseShots = preciseShots;
        this.timeBased = timeBased;
        addRequirements(robot.intake, robot.launcher, robot.turret);
    }

    @Override
    public void initialize() {
        robot.launcher.setActiveControl(true);
        robot.launcher.setRamp(true);

        timer.reset();
    }

    @Override
    public void execute() {
//        RobotLog.ww("ClearLaunch", "Running");

        if (timeBased) {
            if (preciseShots && ((int) timer.milliseconds() % 500) > 200) {
                robot.intake.setIntake(Intake.MotorState.STOP);
            } else {
                robot.intake.setIntake(Intake.MotorState.TRANSFER);
            }
        } else {
            if (!preciseShots || robot.launcher.launchValid()) {
                robot.intake.setIntake(Intake.MotorState.TRANSFER);
            } else {
                robot.intake.setIntake(Intake.MotorState.STOP);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (Constants.OP_MODE_TYPE.equals(Constants.OpModeType.TELEOP) && !Intake.keepIntakeOn) {
            robot.intake.setIntake(Intake.MotorState.STOP);
        } else {
            // TODO: Add distance sensor checking for auto retry command
        }

//        RobotLog.ww("ClearLaunch done, interrupted", String.valueOf(interrupted));

        robot.launcher.setRamp(false);
        robot.launcher.setActiveControl(false);
        robot.readyToLaunch = true;
    }

    @Override
    public boolean isFinished() {
        double endTime;
        if (!preciseShots) {
            endTime = 867;
        } else {
            if (timeBased) {
                endTime = 2000;
            } else {
                endTime = 1467;
            }
        }
        return !robot.readyToLaunch || (timer.milliseconds() > endTime); // TODO: replace with real end condition of the command
    }
}
