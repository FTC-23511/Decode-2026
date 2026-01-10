package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.ANGLE_CONTROL;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.GOAL_LOCK_CONTROL;
import static org.firstinspires.ftc.teamcode.globals.Constants.AIMBOT_COEFFICIENTS;
import static org.firstinspires.ftc.teamcode.globals.Constants.DISTANCE_UNIT;
import static org.firstinspires.ftc.teamcode.globals.Constants.GOAL_POSE;
import static org.firstinspires.ftc.teamcode.globals.Constants.LAUNCHER_HEIGHT;
import static org.firstinspires.ftc.teamcode.globals.Constants.TARGET_HEIGHT;

import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.MathFunctions;
import org.firstinspires.ftc.teamcode.globals.Robot;

public class MovingAim extends CommandBase {
    final double predictTime = 0.5;
    Robot robot;
    final double inchesPerMeters = 39.3701;
    final double BALL_RADIUS = 2.5;
    double[] errorsAngleVelocity;

    MathFunctions.ShootingMath math;

    public MovingAim() {
        robot = Robot.getInstance();

        robot.readyToLaunch = false;

        final Pose2d goalPose = GOAL_POSE();
        Position goalPosition = new Position(DISTANCE_UNIT, goalPose.getX(), goalPose.getY(), TARGET_HEIGHT * inchesPerMeters, 0);
        math = new MathFunctions.ShootingMath(goalPosition, BALL_RADIUS, LAUNCHER_HEIGHT * inchesPerMeters);

        addRequirements(robot.launcher, robot.turret, robot.drive, robot.intake);
    }


    public void initialize() {
        robot.intake.setIntake(Intake.MotorState.STOP);

        ((PIDFController) robot.drive.follower.headingController).setCoefficients(AIMBOT_COEFFICIENTS);

        if (!Turret.turretState.equals(GOAL_LOCK_CONTROL)) {
            robot.turret.setTurret(GOAL_LOCK_CONTROL, 0);
        }
    }

    public void execute() {
        predictSet();
        if (robot.launcher.flywheelReady() && robot.turret.readyToLaunch() && robot.launcher.hoodReady()) {
            robot.readyToLaunch = true;
        }

    }

    public void end(boolean interrupted) {
        if (interrupted) {
            robot.turret.setTurret(Turret.TurretState.OFF, 0);
            robot.launcher.setActiveControl(false);
            robot.readyToLaunch = false;
        }
    }

    public boolean isFinished() {
        return robot.readyToLaunch;
    }

    private void predictSet() {
        Pose2d robotPose = robot.drive.getPose();
        ChassisSpeeds robotSpeed = robot.drive.swerve.getTargetVelocity();

        MathFunctions.ShootingMath.PredictResult values = math.predict(robotPose, robotSpeed);
        //inch/second to meter to seconds
        robot.launcher.setFlywheel(values.flyWheelSpeed, true);
        //set hood angle to degrees in the right range
        robot.launcher.setHood(90 - Math.toDegrees(values.hoodAngle));
        // set turret angle to robot centric and to radians
        robot.turret.setTurret(ANGLE_CONTROL, MathUtils.normalizeRadians(values.turretAngle, false));
        RobotLog.aa("robotPose", String.valueOf(robotPose));
        RobotLog.aa("robotSpeed", String.valueOf(robotSpeed));
        RobotLog.aa("PredictResult", String.valueOf(values));

    }
}
