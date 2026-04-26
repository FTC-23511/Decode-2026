package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

public class SlowClearLaunch extends CommandBase {
    private final Robot robot;
    private final ElapsedTime endTimer;
    private final ElapsedTime shotTimer;

    /**
     * Command to slowly, but more precisely, clear out current balls inside the robot while driving
     */
    public SlowClearLaunch() {
        robot = Robot.getInstance();
        endTimer = new ElapsedTime();
        shotTimer = new ElapsedTime();


        addRequirements(robot.intake, robot.launcher, robot.turret);
    }

    @Override
    public void initialize() {
        if (robot.readyToLaunch) {
            robot.launcher.setActiveControl(true);
        }
        robot.launcher.setRamp(true);

        shotTimer.reset();
        endTimer.reset();
    }

    @Override
    public void execute() {
        if (robot.launcher.launchValid()) {
            robot.intake.setIntake(Intake.MotorState.TRANSFER);
            robot.launcher.setTransfer(true);
        } else {
            robot.intake.setIntake(Intake.MotorState.STOP);
            robot.launcher.setTransfer(false);
        }

    }

    @Override
    public void end(boolean interrupted) {
        if (Constants.OP_MODE_TYPE.equals(Constants.OpModeType.TELEOP)) {
            robot.intake.setIntake(Intake.MotorState.STOP);
            robot.turret.setTurret(Turret.TurretState.GOAL_LOCK_CONTROL, 0);
        } else {
            // TODO: Add distance sensor checking for auto retry command
        }

//        RobotLog.ww("ClearLaunch done, interrupted", String.valueOf(interrupted));

        robot.launcher.setRamp(false);
        robot.readyToLaunch = true;
    }

    @Override
    public boolean isFinished() {
        return endTimer.milliseconds() > 1367;
    }
}