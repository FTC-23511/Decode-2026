package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

public class AlignWithAprilTagCommand extends CommandBase {
    private final Robot robot = Robot.getInstance();
    private final Limelight3A limelight;

    // P-Controller Gain: Adjust this to change how aggressively the robot turns
    private static final double P_GAIN = 0.03;
    private static final double TOLERANCE = 1.0; // Degrees

    public AlignWithAprilTagCommand(Limelight3A limelight) {
        this.limelight = limelight;
        // Require the drive subsystem so other drive commands don't interfere
        addRequirements(robot.drive);
    }

    @Override
    public void initialize() {
        limelight.pipelineSwitch(0); // Ensure AprilTag pipeline is active
    }

    @Override
    public void execute() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx(); // Horizontal offset from crosshair in degrees

            // Simple P-loop for rotation
            double turnSpeed = tx * P_GAIN;

            // Apply the turn speed while keeping X and Y translation at zero
            robot.drive.swerve.updateWithTargetVelocity(
                    new ChassisSpeeds(0, 0, turnSpeed)
            );
        } else {
            // If target is lost, stop moving
            robot.drive.swerve.updateWithTargetVelocity(new ChassisSpeeds(0, 0, 0));
        }
    }

    @Override
    public boolean isFinished() {
        LLResult result = limelight.getLatestResult();
        // Finish if we are within the tolerance
        return result != null && result.isValid() && Math.abs(result.getTx()) < TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when finished or interrupted
        robot.drive.swerve.updateWithTargetVelocity(new ChassisSpeeds(0, 0, 0));
    }
}
