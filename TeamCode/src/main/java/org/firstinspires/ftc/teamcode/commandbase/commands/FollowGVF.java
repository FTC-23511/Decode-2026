package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.gvf.Path;
import org.firstinspires.ftc.teamcode.globals.Robot;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;

public class FollowGVF extends CommandBase {
    private final Path path;
    private final Robot robot = Robot.getInstance();

    public FollowGVF(Path path) {
        this.path = path;
        // Require the drive subsystem so we don't overlap drive commands
        addRequirements(robot.drive);
    }

    @Override
    public void initialize() {
        // Hands the path to your Drive subsystem.
        // Your Drive.update() method takes over from here!
        robot.drive.setPath(path);
    }

    @Override
    public void execute() {
        // Leave this blank!
        // Your Drive subsystem's periodic() loop is already calling Drive.update() constantly.
    }

    @Override
    public boolean isFinished() {
        return robot.drive.driveState == Drive.DriveState.WAIT ||
                robot.drive.driveState == Drive.DriveState.BRAKE;
    }
}