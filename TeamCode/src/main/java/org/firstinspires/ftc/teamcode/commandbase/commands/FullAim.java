package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.util.ArrayList;

public class FullAim extends CommandBase {
    private final Robot robot;
    private final ElapsedTime timer;
    private double aimIndex = 0;
    /**
     * 0 = initial state
     * 1 = moving to initial state estimates for turret / launcher based off pinpoint, lasts until camera sees ATag or timeout
     * 2 = drivetrain stops and turret compensates for remaining drivetrain error, lasts until turret is at target
     * 3 = just waiting for hood/flywheel to reach final set states
     */
    private boolean impossible = false;
    double[] errorsAngleVelocity;

    /**
     * Full aimbot command
     */
    public FullAim() {
        robot = Robot.getInstance();
        this.timer = new ElapsedTime();

        addRequirements(robot.launcher, robot.turret, robot.drive, robot.intake);
    }

    @Override
    public void initialize() {
        robot.intake.setIntake(Intake.MotorState.STOP);
        robot.intake.setPivot(Intake.PivotState.HOLD);

        robot.turret.updateTurretPose(null); // clear any prior readings of where turret was

        robot.camera.updateCameraResult(3);
        Pose2d cameraPose = robot.camera.getCameraPose();

        ((PIDFController) robot.drive.follower.headingController).setCoefficients(AIMBOT_COEFFICIENTS);
        if (cameraPose != null) {
            robot.turret.setTurret(Turret.TurretState.TX_CONTROL, robot.camera.getTxOffset(cameraPose));
            aimIndex = 2;

            if (robot.drive.unsureXY) {
                robot.drive.setPose(new Pose2d(cameraPose.getTranslation(), robot.drive.getPose().getRotation()));
                robot.drive.unsureXY = false;
            }
        } else {
            if (robot.drive.unsureXY) {
                robot.drive.setPose(new Pose2d(new Translation2d(), robot.drive.getPose().getRotation()));
            }

            // Preliminary estimates of where drivetrain and turret should face
            Pose2d robotPose = robot.drive.getPose();
            double[] errorsDriveTurret = Turret.angleToDriveTurretErrors(Turret.posesToAngle(robotPose, robot.turret.adjustedGoalPose(robot.turret.getTurretPose())));
            robot.drive.follower.setTarget(robotPose.rotate(errorsDriveTurret[0]));
            robot.turret.setTurret(Turret.TurretState.GOAL_LOCK_CONTROL, errorsDriveTurret[0] + errorsDriveTurret[1]);

            // Preliminary estimate for launcher values (only used for setting flywheel because hood needs to be down)
            errorsAngleVelocity = Launcher.distanceToLauncherValues(GOAL_POSE().minus(robot.drive.getPose()).getTranslation().getNorm() * DistanceUnit.mPerInch);
            robot.launcher.setFlywheel(errorsAngleVelocity[0], false);
            robot.launcher.setHood(errorsAngleVelocity[1]);
            aimIndex = 1;
        }

        timer.reset();
    }

    @Override
    public void execute() {
        // TODO: Consider making aimIndex condition <= 2
        if (aimIndex < 2 && OP_MODE_TYPE.equals(OpModeType.TELEOP)) {
            robot.drive.swerve.updateWithTargetVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            robot.drive.follower.calculate(robot.drive.getPose()),
                            robot.drive.getPose().getRotation()
                    )
            );
        } else {
            robot.drive.swerve.updateWithXLock();
        }

        RobotLog.aa("aimIndex", String.valueOf(aimIndex));
        robot.camera.updateCameraResult(3);
        Pose2d cameraPose = robot.camera.getCameraPose();

        if (robot.drive.unsureXY && cameraPose != null) {
            robot.drive.setPose(new Pose2d(cameraPose.getTranslation(), robot.drive.getPose().getRotation()));
            robot.drive.unsureXY = false;
        }

        if (aimIndex == 1) {
            if (cameraPose != null) {
                robot.turret.setTurret(Turret.TurretState.TX_CONTROL, robot.camera.getTxOffset(cameraPose));
                aimIndex = 2;
                timer.reset();
            } else if (robot.turret.readyToLaunch() && timer.milliseconds() >= 2000) {
                // TODO: add code to deal with case where if we don't see ATag even if turret is aimed correctly (wiggle or do a full spin or time out)
                // NOTE: implementation should only do extreme measures of rotating if its consistently unable to find an ATag and not a one off loop
            } else {
                Pose2d robotPose = robot.drive.getPose();
                double[] errorsDriveTurret = Turret.angleToDriveTurretErrors(Turret.posesToAngle(robotPose, robot.turret.adjustedGoalPose(robot.turret.getTurretPose())));
                robot.drive.follower.setTarget(robotPose.rotate(errorsDriveTurret[0]));
                robot.turret.setTurret(Turret.TurretState.GOAL_LOCK_CONTROL, errorsDriveTurret[0] + errorsDriveTurret[1]);
            }
        }

        if (aimIndex == 2) {
            if (cameraPose != null) {
                timer.reset();
            } else if (timer.milliseconds() > 1000) {
                aimIndex = 1;
            }

            if (robot.turret.readyToLaunch()) {
                RobotLog.aa("done with aimbot", String.valueOf(robot.camera.getTargetDegrees()[0]));
                robot.turret.setTurret(Turret.TurretState.OFF, robot.turret.getPosition()); // lock turret to current position
                errorsAngleVelocity = Launcher.distanceToLauncherValues(robot.turret.adjustedGoalPose().minus(robot.turret.getTurretPose()).getTranslation().getNorm() * DistanceUnit.mPerInch + DISTANCE_BS); // TODO: REPLACE BS -0.1
                if (Double.isNaN(errorsAngleVelocity[0])) {
                    impossible = true;
                } else {
                    robot.launcher.setFlywheel(errorsAngleVelocity[0], true);
                    robot.launcher.setHood(errorsAngleVelocity[1]);
                }
                aimIndex = 3;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (impossible && !interrupted) {
            // TODO: Come up with a better way to deal with this
            robot.launcher.setFlywheel(LAUNCHER_CLOSE_VELOCITY, false);
            robot.launcher.setHood(MIN_HOOD_ANGLE);
        }

        if (OP_MODE_TYPE.equals(OpModeType.TELEOP)) {
            ((PIDFController) robot.drive.follower.headingController).setCoefficients(TELEOP_HEADING_COEFFICIENTS);
        }

        robot.readyToLaunch = !interrupted && !impossible;
    }

    @Override
    public boolean isFinished() {
        return impossible || (aimIndex == 3 && robot.launcher.flywheelReady());
    }
}
