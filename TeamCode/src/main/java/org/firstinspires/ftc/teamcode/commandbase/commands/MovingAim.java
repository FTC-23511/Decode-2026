package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.ANGLE_CONTROL;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret.TurretState.GOAL_LOCK_CONTROL;
import static org.firstinspires.ftc.teamcode.globals.Constants.AIMBOT_COEFFICIENTS;
import static org.firstinspires.ftc.teamcode.globals.Constants.DISTANCE_UNIT;
import static org.firstinspires.ftc.teamcode.globals.Constants.GOAL_POSE;
import static org.firstinspires.ftc.teamcode.globals.Constants.LAUNCHER_HEIGHT;
import static org.firstinspires.ftc.teamcode.globals.Constants.TARGET_HEIGHT;

import com.qualcomm.robotcore.util.ElapsedTime;
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
    private enum AimStateType {
        AIMING,
        TRANSFERRING,
        LAUNCHING,
        FINISHED,
    }


    Robot robot;
    final double inchesPerMeters = 39.3701;
    final double BALL_RADIUS = 2.5;
    final int MAX_LAUNCH_TIMES = 3;
    final double MIN_TRANSFER_TIME_MS = 500;
    final double MAX_LAUNCH_TIME_MS = 1000;
    final double MAX_EXECUTION_TIME_MS = 10000;
    final double FLYWHEEL_VELOCITY_LOSS_RATE = 0.01;
    final double TURRET_IDLE_VELOCITY = 0.05;

    MathFunctions.ShootingMath math;

    AimStateType aimState = AimStateType.AIMING;

    private final ElapsedTime timer;

    private double endExecutionTime = 0;
    private double transferEndTime = 0;
    private double launchEndTime = 0;
    private double flywheelLaunchVelocity = 0;
    private int launchTimes = 0;

    public MovingAim() {
        robot = Robot.getInstance();

        robot.readyToLaunch = false;

        final Pose2d goalPose = GOAL_POSE();
        Position goalPosition = new Position(DISTANCE_UNIT, goalPose.getX(), goalPose.getY(), TARGET_HEIGHT * inchesPerMeters, 0);
        math = new MathFunctions.ShootingMath(goalPosition, BALL_RADIUS, LAUNCHER_HEIGHT * inchesPerMeters);

        timer = new ElapsedTime();

        addRequirements(robot.launcher, robot.turret, robot.drive, robot.intake);
    }


    public void initialize() {
        launchTimes = 0;
        aimState = AimStateType.AIMING;
        endExecutionTime = timer.milliseconds() + MAX_EXECUTION_TIME_MS;

        robot.intake.setIntake(Intake.MotorState.STOP);

        ((PIDFController) robot.drive.follower.headingController).setCoefficients(AIMBOT_COEFFICIENTS);

        if (!Turret.turretState.equals(GOAL_LOCK_CONTROL)) {
            robot.turret.setTurret(GOAL_LOCK_CONTROL, 0);
        }
    }

    public void execute() {
        final AimStateType currentState = aimState;

        if (currentState != AimStateType.FINISHED) {
            predictSet();
        }

        switch (aimState) {
            case AIMING:
                if (isReadyToLaunch()) {
                    aimState = AimStateType.TRANSFERRING;
                    RobotLog.aa("MovingAimState", "Changing state 0: " + currentState + " --> " + aimState);
                    startTransfer();
                }
                break;

            case TRANSFERRING:
                if (timer.milliseconds() >= transferEndTime && isReadyToLaunch()) {
                    aimState = AimStateType.LAUNCHING;
                    RobotLog.aa("MovingAimState", "Changing state 1: " + currentState + " --> " + aimState);
                    startLaunch();
                }
                break;

            case LAUNCHING:
                if (hasBallLaunched()) {
                    stopLaunch();
                    aimState = !isFinished() ? AimStateType.AIMING : AimStateType.FINISHED;
                    RobotLog.aa("MovingAimState", "Changing state 2: " + currentState + " --> " + aimState);
                }
                break;

            default:
                launchTimes = MAX_LAUNCH_TIMES;
                end(false);
                break;
        }

//        if (currentState != aimState) {
            RobotLog.aa("MovingAimState", "Current state = " + currentState + ", Next State = " + aimState + ", Launch Times = " + launchTimes);
//        }
    }

    public void end(boolean interrupted) {
        RobotLog.aa("MovingAimState", "end: interrupted = " + interrupted);
        robot.turret.setTurret(Turret.TurretState.OFF, 0);
        robot.launcher.setActiveControl(false);
        robot.intake.setIntake(Intake.MotorState.STOP);
        robot.turret.setTurret(Turret.TurretState.GOAL_LOCK_CONTROL, 0);
        robot.launcher.setFlywheel(0, false);

        robot.readyToLaunch = false;
    }

    public boolean isFinished() {
        final boolean finished = aimState == AimStateType.FINISHED;
        final boolean launchedAllBalls = launchTimes >= MAX_LAUNCH_TIME_MS;
        final boolean timeout = timer.milliseconds() >= endExecutionTime;
        RobotLog.aa("MovingAimState", "finished = " + finished + ", launchedAllBalls = " + launchedAllBalls + ", timeout = " + timeout);
        return (aimState == AimStateType.FINISHED) || (launchTimes >= MAX_LAUNCH_TIMES) || timer.milliseconds() >= endExecutionTime;
    }

    private boolean isReadyToLaunch() {
        final boolean flywheelReady = robot.launcher.flywheelReady();
        final boolean turretReady = robot.turret.readyToLaunch();
        final boolean launchValid = robot.launcher.launchValid();

        RobotLog.aa("MovingAimState", "flywheelReady = " + flywheelReady + ", turrentReady = " + turretReady + ", launchValid = " + launchValid);

        return flywheelReady && turretReady;

//        return robot.launcher.flywheelReady() && robot.turret.readyToLaunch() && robot.launcher.launchValid() && Math.abs(robot.turretEncoder.getCorrectedVelocity()) <= TURRET_IDLE_VELOCITY;
        //return robot.launcher.flywheelReady() && robot.turret.readyToLaunch() /*&& robot.launcher.launchValid()*/;
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

        // log the aim telemetry
//        RobotLog.aa("MovingAimStatus", "Robot Pose = " + String.valueOf(robotPose) + ", Robot Speed = ", String.valueOf(robotSpeed) + ", Predict Result = " + String.valueOf(values));
        RobotLog.aa("MovingAimState", "Robot Pose = " + String.valueOf(robotPose));
        RobotLog.aa("MovingAimState", "Robot Speed = " + String.valueOf(robotSpeed));
        RobotLog.aa("MovingAimState", "Predict Result = " + String.valueOf(values));
    }

    private void startTransfer() {
        robot.intake.setIntake(Intake.MotorState.TRANSFER);
        transferEndTime = timer.milliseconds() + MIN_TRANSFER_TIME_MS;
    }

    private void startLaunch() {
        robot.readyToLaunch = true;
        flywheelLaunchVelocity = robot.launchEncoder.getCorrectedVelocity();
        launchEndTime = timer.milliseconds() + MAX_LAUNCH_TIME_MS;
        robot.launcher.setRamp(true);
    }

    private void stopLaunch() {
        robot.launcher.setRamp(false);
        robot.intake.setIntake(Intake.MotorState.STOP);
        robot.readyToLaunch = false;
        launchTimes++;
    }

    private boolean hasBallLaunched() {
        if (timer.milliseconds() >= launchEndTime) {
            return true;
        }

        final double flywheelVelocity = robot.launchEncoder.getCorrectedVelocity();
        if (flywheelVelocity >= flywheelLaunchVelocity) {
            return false;
        }

        final double rate = (flywheelLaunchVelocity - flywheelVelocity) / flywheelLaunchVelocity;
        return rate >= FLYWHEEL_VELOCITY_LOSS_RATE;
    }
}
