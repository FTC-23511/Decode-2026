package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

public class ClearLaunch extends CommandBase {
    private final Robot robot;
    private final ElapsedTime timer;
    private final boolean preciseShots;
    private int index = 0;

    /**
     * Assumes {@link FullAim} has already been performed or the robot is already aimed
     * Command to clear out current balls inside the robot
     */
    public ClearLaunch() {
        this(false);
    }

    /**
     * Assumes {@link FullAim} has already been performed or the robot is already aimed
     * Command to clear out current balls inside the robot
     * @param preciseShots whether the feeder should wait until flywheel is at target velocity for precise shots or just rapid fire
     */
    public ClearLaunch(boolean preciseShots) {
        robot = Robot.getInstance();
        timer = new ElapsedTime();
        this.preciseShots = preciseShots;
        addRequirements(robot.intake, robot.launcher, robot.turret);
    }

    @Override
    public void initialize() {
        if (robot.readyToLaunch) {
//            robot.turret.setTurret(Turret.TurretState.ANGLE_CONTROL, robot.turret.getPosition());
            robot.launcher.setRamp(true);
            robot.launcher.setActiveControl(true);
            robot.intake.setIntake(Intake.MotorState.TRANSFER);
            robot.intake.setPivot(Intake.PivotState.TRANSFER);
        }
        timer.reset();
    }

    @Override
    public void execute() {
        // TODO: Add code to auto launch third ball that sometimes gets stuck
        if (preciseShots && !robot.launcher.flywheelReady()) {
            robot.intake.setIntake(Intake.MotorState.STOP);
        } else if (robot.intake.intakeJammed || robot.intakeMotor.isOverCurrent()) {
            // do nothing and let intake auto unjam
        } else {
            robot.intake.setIntake(Intake.MotorState.TRANSFER);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (Constants.OP_MODE_TYPE.equals(Constants.OpModeType.TELEOP)) {
            robot.intake.setIntake(Intake.MotorState.STOP);
        } else {
            // TODO: Add distance sensor checking for auto retry command
        }

        robot.launcher.setRamp(false);
        robot.turret.setTurret(Turret.TurretState.OFF, 0);
        robot.launcher.setActiveControl(false);
        robot.readyToLaunch = false;
    }

    @Override
    public boolean isFinished() {
        return !robot.readyToLaunch || (timer.milliseconds() > 2000); // TODO: replace with real end condition of the command
    }
}
