package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.vision.VisionPortal.CameraState.STREAMING;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.globals.Robot;

public class CameraLocalization extends CommandBase {
    private final Robot robot;

    public CameraLocalization(double n) {
        robot = Robot.getInstance();

        addRequirements(); // TODO: replace with requirements of the command
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true; // TODO: replace with end condition of the command
    }
}
