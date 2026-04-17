package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.globals.Constants.ENABLE_ZONE_CONTROL;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

public class ContinuousClearLaunch extends CommandBase {
    private final Robot robot;
    private final ElapsedTime timer;

    /**
     * Command to clear out current balls inside the robot while driving
     */
    public ContinuousClearLaunch() {
        robot = Robot.getInstance();
        timer = new ElapsedTime();
        addRequirements(robot.intake, robot.launcher, robot.turret);
    }

    @Override
    public void initialize() {
        RobotLog.ww("ContinuousClearLaunch", "Initialized");
        robot.launcher.setActiveControl(true);
        robot.launcher.setRamp(true);
        robot.turret.setTurret(Turret.TurretState.GOAL_LOCK_CONTROL, 0);

        timer.reset();
    }

    @Override
    public void execute() {
        if (robot.launcher.launchWillBeValid() && (!ENABLE_ZONE_CONTROL || Drive.robotInZone(robot.drive.getPose()))) {
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

        robot.launcher.setRamp(false);
        robot.launcher.setActiveControl(false);
        RobotLog.ww("ContinuousClearLaunch", "Ended");
    }

    @Override
    public boolean isFinished() {
        return !robot.readyToLaunch;
    }
}