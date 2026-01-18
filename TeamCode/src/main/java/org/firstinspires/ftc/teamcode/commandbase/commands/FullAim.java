package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.MathFunctions;
import org.firstinspires.ftc.teamcode.globals.Robot;

public class FullAim extends CommandBase {
    private final Robot robot;
    private final ElapsedTime timer;
    private double aimIndex = 0;
    /**
     * 0 = initial state, ends in initialize()
     * 1 = moving to initial state estimates for turret / launcher based off pinpoint, ends when turret is aimed at goal
     * 2 = just waiting for hood/flywheel to reach final set states
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
        robot.launcher.setRamp(false);

        ((PIDFController) robot.drive.follower.headingController).setCoefficients(AIMBOT_COEFFICIENTS);

        if (!Turret.turretState.equals(Turret.TurretState.GOAL_LOCK_CONTROL)) {
            robot.turret.setTurret(Turret.TurretState.GOAL_LOCK_CONTROL, 0);
        }

        // Preliminary estimate for launcher values
        errorsAngleVelocity = MathFunctions.distanceToLauncherValues(GOAL_POSE().minus(robot.drive.getPose()).getTranslation().getNorm() * DistanceUnit.mPerInch);
        robot.launcher.setFlywheel(errorsAngleVelocity[0], false);
        robot.launcher.setHood(errorsAngleVelocity[1]);
        aimIndex = 1;

        timer.reset();
    }

    @Override
    public void execute() {
        if (!OP_MODE_TYPE.equals(OpModeType.AUTO)) {
            robot.drive.swerve.updateWithXLock();
        }

        RobotLog.aa("aimIndex", String.valueOf(aimIndex));

        if (robot.turret.readyToLaunch()) {
            RobotLog.aa("turret aimbot done", String.valueOf(robot.turret.readyToLaunch()));
//                robot.turret.setTurret(Turret.TurretState.OFF, robot.turret.getPosition()); // lock turret to current position
            robot.intake.setIntake(Intake.MotorState.TRANSFER);
            errorsAngleVelocity = MathFunctions.distanceToLauncherValues(robot.turret.adjustedGoalPose().minus(robot.turret.getTurretPose()).getTranslation().getNorm() * DistanceUnit.mPerInch);

            if (Double.isNaN(errorsAngleVelocity[0])) {
                impossible = true;
            } else {
                robot.launcher.setFlywheel(errorsAngleVelocity[0], true);
                robot.launcher.setHood(errorsAngleVelocity[1]);
            }
            aimIndex = 2;
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
        } else {
            ((PIDFController) robot.drive.follower.headingController).setCoefficients(HEADING_COEFFICIENTS);
        }

        robot.turret.setTurret(Turret.TurretState.OFF, 0);

        robot.readyToLaunch = true;
    }

    @Override
    public boolean isFinished() {
        return impossible || (aimIndex == 2 && robot.launcher.flywheelReady() && robot.turret.readyToLaunch());
    }
}
