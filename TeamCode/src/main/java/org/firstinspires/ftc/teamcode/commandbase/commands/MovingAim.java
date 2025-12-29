package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.ANGLE_CONTROL;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.GOAL_LOCK_CONTROL;
import static org.firstinspires.ftc.teamcode.globals.Constants.AIMBOT_COEFFICIENTS;
import static org.firstinspires.ftc.teamcode.globals.Constants.GOAL_POSE;
import static org.firstinspires.ftc.teamcode.globals.Constants.LAUNCHER_CLOSE_VELOCITY;
import static org.firstinspires.ftc.teamcode.globals.Constants.LAUNCHER_HEIGHT;
import static org.firstinspires.ftc.teamcode.globals.Constants.MIN_HOOD_ANGLE;
import static org.firstinspires.ftc.teamcode.globals.Constants.TARGET_HEIGHT;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    private final ElapsedTime timer;
    Position target;

    MathFunctions.ShootingMath math;
    public MovingAim() {
        robot = Robot.getInstance();
        timer = new ElapsedTime();

        robot.readyToLaunch = false;

        final Pose2d goalPose = GOAL_POSE();
        Position goalPosition = new Position(DistanceUnit.METER, goalPose.getX() * inchesPerMeters, goalPose.getY() * inchesPerMeters, TARGET_HEIGHT, 0);
        math = new MathFunctions.ShootingMath(goalPosition, LAUNCHER_HEIGHT, BALL_RADIUS);

        addRequirements(robot.launcher, robot.turret, robot.drive, robot.intake);
    }


    public void initialize() {
        robot.intake.setIntake(Intake.MotorState.STOP);
        robot.intake.setPivot(Intake.PivotState.HOLD);

        robot.turret.updateTurretPose(null); // clear any prior readings of where turret was

        ((PIDFController) robot.drive.follower.headingController).setCoefficients(AIMBOT_COEFFICIENTS);

        if (!Turret.turretState.equals(GOAL_LOCK_CONTROL)) {
            robot.turret.setTurret(GOAL_LOCK_CONTROL, 0);
        }

        timer.reset();
    }
    public void execute() {
        predictSet();
            if(robot.launcher.flywheelReady()&&robot.turret.readyToLaunch()) {
                robot.readyToLaunch = true;
            }

    }

    public void end(boolean interrupted){
        if (!interrupted) {
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
        robot.launcher.setFlywheel(values.flyWheelSpeed, true);
        robot.launcher.setHood(values.hoodAngle);
        robot.turret.setTurret(ANGLE_CONTROL,values.turretAngle);
    }
}
