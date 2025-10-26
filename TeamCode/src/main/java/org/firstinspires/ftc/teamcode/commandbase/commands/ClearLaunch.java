package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

public class ClearLaunch extends CommandBase {
    private final Robot robot;
    private ElapsedTime timer;
    private boolean targetStateSolved = false;

    /**
     * Assumes {@link FullAim} has already been performed
     * Command to clear out current balls inside the robot
     */
    public ClearLaunch() {
        robot = Robot.getInstance();
        addRequirements(robot.intake, robot.launcher, robot.turret);
        timer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        robot.turret.setTurret(Turret.TurretState.ANGLE_CONTROL, robot.turret.getPosition());
        robot.launcher.setActiveControl(true);
        robot.launcher.setRamp(true);
        robot.intake.setIntake(Intake.MotorState.TRANSFER);
        robot.intake.setPivot(Intake.PivotState.TRANSFER);
        timer.reset();
    }

    @Override
    public void execute() {
        // TODO: Add code to auto launch third ball that sometimes gets stuck
    }

    @Override
    public void end(boolean interrupted) {
        if (Constants.OP_MODE_TYPE.equals(Constants.OpModeType.TELEOP)) {
            robot.intake.setIntake(Intake.MotorState.STOP);
        }

        robot.launcher.setRamp(false);
        robot.turret.setTurret(Turret.TurretState.OFF, 0);
        robot.launcher.setActiveControl(false);
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > 2000; // TODO: replace with real end condition of the command
    }
}
