package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.globals.Robot;

public class DriveTo extends CommandBase {
    private final Robot robot;
    private final Pose2d target;
    private final double maxPower;
    private final double previousMaxPower;

    public DriveTo(Pose2d pose) {
        this(pose, Robot.getInstance().drive.swerve.getMaxSpeed());
    }

    public DriveTo(Pose2d pose, double maxPower) {
        target = pose;
        robot = Robot.getInstance();
        previousMaxPower = robot.drive.swerve.getMaxSpeed();
        this.maxPower = maxPower;
        addRequirements(robot.drive);
    }

    @Override
    public void initialize() {
        robot.drive.follower.setTarget(target);
    }

    @Override
    public void execute() {
        if (maxPower != previousMaxPower) {
            robot.drive.swerve.setMaxSpeed(maxPower);
        }

        // 1. Get FIELD-RELATIVE speeds (X=Global X, Y=Global Y)
        // 2. Convert to ROBOT-RELATIVE speeds (X=Forward, Y=Left)
        // This requires the current robot heading.
        // 3. Send to Swerve
        robot.drive.swerve.updateWithTargetVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                robot.drive.follower.calculate(robot.drive.getPose()),
                robot.drive.getPose().getRotation()
        ));
    }

    @Override
    public boolean isFinished() {
        return robot.drive.follower.atTarget();
    }

    @Override
    public void end(boolean interrupted) {
        robot.drive.swerve.setMaxSpeed(previousMaxPower);
    }
}
