package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.globals.Robot;

public class DriveTo extends CommandBase {
    private final Robot robot;
    private final Pose2d target;

    public DriveTo(Pose2d pose) {
        target = pose;
        robot = Robot.getInstance();
        addRequirements(robot.drive);
    }

    @Override
    public void initialize() {
        robot.drive.follower.setTarget(target);
    }

    @Override
    public void execute() {
        robot.drive.swerve.updateWithTargetVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        robot.drive.follower.calculate(robot.drive.getPose()),
                        robot.drive.getPose().getRotation()
                )
        );
    }

    @Override
    public boolean isFinished() {
        return robot.drive.follower.atTarget();
    }
}
