package org.firstinspires.ftc.teamcode.opmode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

@Autonomous(name = "Drive Forward 2 Feet", group = "RobotCentric")
public class DriveForwardAuto extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private double startY = 0;

    @Override
    public void initialize() {
        // 1. Set OpMode Type to AUTO for pinpoint polling rates
        Constants.OP_MODE_TYPE = OpModeType.AUTO;

        // 2. Initialize Hardware
        robot.init(hardwareMap);
        robot.initHasMovement();

        // 3. Define the sequence
        schedule(new SequentialCommandGroup(
                // Step A: Reset Pinpoint and record starting Y (Forward is Y in Pinpoint default)
                new InstantCommand(() -> {
                    robot.drive.setPose(new Pose2d(0, 0, 0));
                    startY = robot.drive.getPose().getY();
                }),

                // Step B: Drive forward at 40% speed
                // We calculate ChassisSpeeds(VX, VY, Omega).
                // For robot-centric forward, we set VX.
                new InstantCommand(() -> {
                    robot.drive.swerve.updateWithTargetVelocity(
                            new ChassisSpeeds(MAX_DRIVE_VELOCITY * 0.4, 0, 0)
                    );
                }),

                // Step C: Wait until we have moved 24 inches (2 feet)
                // Math.abs handles both directions, but here we drive +X
                new WaitUntilCommand(() -> Math.abs(robot.drive.getPose().getX() - 0) >= 24.0),

                // Step D: Stop the robot
                new InstantCommand(() -> {
                    robot.drive.swerve.updateWithTargetVelocity(new ChassisSpeeds(0, 0, 0));
                }),

                // Step E: Save the final pose to Constants for TeleOp Handoff
                new InstantCommand(() -> {
                    Constants.END_POSE = robot.drive.getPose();
                })
        ));
    }
}
