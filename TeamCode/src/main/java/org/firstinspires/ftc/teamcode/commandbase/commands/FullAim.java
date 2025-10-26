package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.FunctionalCommand;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

public class FullAim extends CommandBase {
    private final Robot robot;
    private final ElapsedTime hoodServoMoveTimer;
    private boolean secondaryAim = false;
    private boolean impossible = false;
    double[] errorsAngleVelocity;

    /**
     * Full aimbot command
     */
    public FullAim() {
        robot = Robot.getInstance();
        this.hoodServoMoveTimer = new ElapsedTime();

        addRequirements(robot.launcher, robot.turret, robot.drive, robot.intake);
    }

    @Override
    public void initialize() {
        robot.intake.setIntake(Intake.MotorState.STOP);
        robot.intake.setPivot(Intake.PivotState.HOLD);

        // Preliminary estimates of where drivetrain and turret should face
        double[] errorsDriveTurret = Turret.angleToDriveTurretErrors(Turret.angleToGoal(robot.drive.getPose()));
        robot.drive.follower.setTarget(robot.drive.getPose().rotate(errorsDriveTurret[0]));
        robot.turret.setTurret(Turret.TurretState.ANGLE_CONTROL, errorsDriveTurret[1]);

        // Preliminary estimate for launcher values (only used for setting flywheel because hood needs to be down)
        errorsAngleVelocity = Launcher.distanceToLauncherValues(Constants.GOAL_POSE().minus(robot.drive.getPose()).getTranslation().getNorm());
        robot.launcher.setFlywheel(errorsAngleVelocity[0], true);
        robot.launcher.setHood(MIN_LL_HOOD_ANGLE); // Put hood down to be able to see the AprilTags
    }

    @Override
    public void execute() {
        robot.drive.swerve.updateWithTargetVelocity(robot.drive.follower.calculate(robot.drive.getPose()));

        if (robot.turret.readyToLaunch() && Turret.turretState.equals(Turret.TurretState.ANGLE_CONTROL) && !secondaryAim) {
            robot.turret.setTurret(Turret.TurretState.LIMELIGHT_CONTROL, 0);
        }

        // TODO: Add code to set targets for turret and condition to set final hood / shooter RPM values
        if (robot.turret.readyToLaunch() && Turret.turretState.equals(Turret.TurretState.LIMELIGHT_CONTROL) && !secondaryAim) {
            errorsAngleVelocity = Launcher.distanceToLauncherValues(Constants.GOAL_POSE().minus(robot.drive.getPose()).getTranslation().getNorm());
            if (Double.isNaN(errorsAngleVelocity[0])) {
                impossible = true;
            } else {
                robot.launcher.setFlywheel(errorsAngleVelocity[0], true);
                hoodServoMoveTimer.reset();
                robot.launcher.setHood(errorsAngleVelocity[1]);
            }
            secondaryAim = true;
            robot.turret.setTurret(Turret.TurretState.OFF, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (impossible) {
//            throw new RuntimeException("Can't see ATags");
            // TODO: Come up with a better way to deal with this
            robot.launcher.setFlywheel(LAUNCHER_CLOSE_VELOCITY, false);
            robot.launcher.setHood(MIN_HOOD_ANGLE);
        }
    }

    @Override
    public boolean isFinished() {
        return impossible || (secondaryAim && robot.launcher.flywheelReady() && robot.turret.readyToLaunch() && robot.drive.follower.atTarget() && hoodServoMoveTimer.milliseconds() > 300);
    }
}
