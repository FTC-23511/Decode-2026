package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Robot;

public class CancelCommand extends CommandBase {
    private final Robot robot;

    /**
     * Cancels all other commands (empty command with requirements for all subsystems)
     * and sets the robot back to a known state
     */
    public CancelCommand() {
        robot = Robot.getInstance();
        addRequirements(robot.drive, robot.turret, robot.intake, robot.launcher);
    }

    @Override
    public void initialize() {
        robot.turret.setTurret(Turret.TurretState.OFF, 0);
        robot.intake.setIntake(Intake.MotorState.STOP);
        robot.intake.setPivot(Intake.PivotState.HOLD);
        robot.launcher.setFlywheel(LAUNCHER_CLOSE_VELOCITY, false);
        robot.launcher.setRamp(false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
