package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

public class FullAim extends CommandBase {
    private final Robot robot;
    private final ElapsedTime timer;
    private double aimIndex = 0;
    /**
     * 0 = initial state
     * 1 = moving to initial state estimates for turret / launcher based off pinpoint
     * 2 = limelight sees ATag and turret switches to LIMELIGHT_CONTROL
     * 2.5 = limelight is still aiming but drivetrain locks once turret gets first reading that it is readyToLaunch
     * 3 = turret is locked back to ANGLE_CONTROL and just waiting for hood/flywheel to reach final set states
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
        ((PIDFController) robot.drive.follower.headingController).setCoefficients(HEADING_COEFFICIENTS);

        robot.intake.setIntake(Intake.MotorState.STOP);
        robot.intake.setPivot(Intake.PivotState.HOLD);

        // Preliminary estimates of where drivetrain and turret should face
        double[] errorsDriveTurret = Turret.angleToDriveTurretErrors(Turret.angleToPose(robot.drive.getPose(), GOAL_POSE()));
        robot.drive.follower.setTarget(robot.drive.getPose().rotate(errorsDriveTurret[0]));
        robot.turret.setTurret(Turret.TurretState.ANGLE_CONTROL, errorsDriveTurret[1]);

        // Preliminary estimate for launcher values (only used for setting flywheel because hood needs to be down)
        errorsAngleVelocity = Launcher.distanceToLauncherValues(Constants.GOAL_POSE().minus(robot.drive.getPose()).getTranslation().getNorm() * DistanceUnit.mPerInch);
        robot.launcher.setFlywheel(errorsAngleVelocity[0], true);
        robot.launcher.setHood(MIN_LL_HOOD_ANGLE); // Put hood down to be able to see the AprilTags
        aimIndex = 1;
    }

    @Override
    public void execute() {
        if (aimIndex <= 2) {
            robot.drive.swerve.updateWithTargetVelocity(robot.drive.follower.calculate(robot.drive.getPose()));
        } else {
            robot.drive.swerve.updateWithXLock();
        }

        if (aimIndex <= 2.5) {
            robot.turret.updateLLResult(5);
        }

        if (aimIndex == 1) {
            if (robot.turret.getLimeLightTargetDegrees() != null) {
                robot.turret.setTurret(Turret.TurretState.LIMELIGHT_CONTROL, robot.turret.limelightInterplut.get(robot.drive.getAngleToGoal(robot.turret.getLimelightPose())));
                aimIndex = 2;
            } else if (robot.turret.readyToLaunch() && robot.turret.llResult == null) {
                // TODO: add code to deal with case where if we don't see ATag despite turret + drivetrain reaching set point
            }
        }

        if (aimIndex == 2) {
            if (robot.turret.readyToLaunch()) {
                aimIndex = 2.5;
                timer.reset();
            } else if (robot.turret.llResult == null) {
                // TODO: add code to deal with case where if we don't see ATag even if drivetrain is rotated (wiggle or do a full spin or time out)
                // NOTE: implementation should only do extreme measures of rotating if its consistently unable to find an ATag and not a one off loop
            }
        }

        if (aimIndex == 2.5 && timer.milliseconds() > 250) {
            if (robot.turret.readyToLaunch()) {
                robot.turret.setTurret(Turret.TurretState.ANGLE_CONTROL, robot.turret.getPosition()); // lock turret to current position
                Pose2d robotPose = robot.turret.getLimelightPose();

                if (robotPose == null) {
                    robotPose = robot.drive.getPose();
                }

                errorsAngleVelocity = Launcher.distanceToLauncherValues(Constants.GOAL_POSE().minus(robotPose).getTranslation().getNorm() * DistanceUnit.mPerInch);
                if (Double.isNaN(errorsAngleVelocity[0])) {
                    impossible = true;
                } else {
                    robot.launcher.setFlywheel(errorsAngleVelocity[0], true);
                    timer.reset();
                    robot.launcher.setHood(errorsAngleVelocity[1]);
                }
                aimIndex = 3;
            } else {
                aimIndex = 2; // go back to moving the drivetrain because turret can't reach position
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

        robot.turret.setTurret(Turret.TurretState.ANGLE_CONTROL, robot.turret.getPosition());
        robot.readyToLaunch = !interrupted && !impossible;
    }

    @Override
    public boolean isFinished() {
        return impossible || (aimIndex == 3 && robot.launcher.flywheelReady() && robot.turret.readyToLaunch() && timer.milliseconds() > 250);
    }
}
