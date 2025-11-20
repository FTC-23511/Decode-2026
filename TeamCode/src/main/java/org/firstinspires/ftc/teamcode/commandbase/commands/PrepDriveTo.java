package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.globals.Robot;

/**
 * Aims all modules at a new target but does not actually move the wheels (applies essentially 0 power)
 */
public class PrepDriveTo extends CommandBase {
    private final Robot robot;
    private final Pose2d target;
    private ElapsedTime timer = new ElapsedTime();

    public PrepDriveTo(Pose2d pose) {
        target = pose;
        robot = Robot.getInstance();
        addRequirements(robot.drive);
    }

    @Override
    public void initialize() {
        robot.drive.follower.setTarget(target);
        timer.reset();
    }

    @Override
    public void execute() {
        robot.drive.swerve.updateWithTargetVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        robot.drive.follower.calculate(robot.drive.getPose()),
                        robot.drive.getPose().getRotation()
                ).scale(0.000001)
        );
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > 300;
    }
}
