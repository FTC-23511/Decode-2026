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
    private final double shotGap;
    private boolean isTransferring = false;
    private int ballCount = 0;

    /**
     * Command to slowly, but more precisely, clear out current balls inside the robot while driving
     */
    public SlowClearLaunch() {
        this(true);
    }

    public SlowClearLaunch(boolean sortMode) {
        robot = Robot.getInstance();
        endTimer = new ElapsedTime();
        shotTimer = new ElapsedTime();

        if (sortMode) {
            shotGap = 200;
        } else {
            shotGap = 0;
        }

        addRequirements(robot.intake, robot.launcher, robot.turret);
    }

    @Override
    public void initialize() {
        if (robot.readyToLaunch) {
            robot.launcher.setActiveControl(true);
        }
        robot.launcher.setRamp(true);

        endTimer.reset();
        shotTimer.reset();
        isTransferring = false;
        ballCount = 0;
    }

    @Override
    public void execute() {
//        RobotLog.ww("ClearLaunch", "Running");

        robot.drive.swerve.updateWithXLock();

        if (!robot.launcher.launchValid()) {
            robot.intake.setIntake(Intake.MotorState.STOP);
            robot.launcher.setTransfer(false);
            shotTimer.reset();
            isTransferring = false;
        } else {
            if (isTransferring) {
                if (shotTimer.milliseconds() >= 100) {
                    robot.intake.setIntake(Intake.MotorState.STOP);
                    robot.launcher.setTransfer(false);
                    shotTimer.reset();
                    isTransferring = false;
                    ballCount++;
                }
            } else {
                // If it's the first ball, don't wait for the gap
                if (shotTimer.milliseconds() >= (ballCount == 0 ? 0 : shotGap)) {
                    robot.intake.setIntake(Intake.MotorState.TRANSFER);
                    robot.launcher.setTransfer(true);
                    shotTimer.reset();
                    isTransferring = true;
                }
            }
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
        return (ballCount >= 3 || endTimer.milliseconds() > 2000);
    }
}