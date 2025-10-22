package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Robot;

public class ReadyToLaunch extends CommandBase {
    private final Robot robot;
    private boolean targetStateSolved = false;

    /**
     * Full aimbot command
     */
    public ReadyToLaunch() {
        robot = Robot.getInstance();
        addRequirements(robot.intake, robot.launcher, robot.turret, robot.drive);
    }

    @Override
    public void initialize() {
        robot.turret.setActiveControl(true);
        robot.launcher.setActiveControl(true);
        robot.intake.setIntake(Intake.MotorState.STOP);
        robot.launcher.setRamp(true);
        robot.launcher.setHood(MIN_LL_HOOD_ANGLE); // Put hood down to be able to see the AprilTags

        // Preliminary estimates of where drivetrain and turret should face
        double[] errorsDriveTurret = Turret.angleToDriveTurretErrors(Turret.angleToGoal(robot.drive.getPose()));
        robot.drive.follower.setTarget(robot.drive.getPose().rotate(errorsDriveTurret[0]));
        robot.turret.setTarget(errorsDriveTurret[1], true);
    }

    @Override
    public void execute() {
        // TODO: Add code to set targets for turret and launcher flywheel as well as set targetStateSolved to true once target state for the robot has been determined
    }

    @Override
    public void end(boolean interrupted) {
        // TODO: Set hood to desired angle
    }

    @Override
    public boolean isFinished() {
        return targetStateSolved && robot.launcher.flywheelReady() && robot.turret.readyToLaunch();
    }
}
