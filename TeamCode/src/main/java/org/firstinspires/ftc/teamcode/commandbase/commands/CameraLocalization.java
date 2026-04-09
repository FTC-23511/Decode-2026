package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.vision.VisionPortal.CameraState.STREAMING;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.globals.Robot;

public class CameraLocalization extends CommandBase {
    private final Robot robot;
    private final int n;
    private boolean finished = false;

    public CameraLocalization(int n) {
        robot = Robot.getInstance();
        this.n = n;

        addRequirements(robot.camera);
    }

    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public void execute() {
        if (robot.camera.relocalizeArducam(n)) {
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
