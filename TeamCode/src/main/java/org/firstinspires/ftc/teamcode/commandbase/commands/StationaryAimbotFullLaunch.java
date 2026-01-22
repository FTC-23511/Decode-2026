package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.globals.Robot;

public class StationaryAimbotFullLaunch extends SequentialCommandGroup {
    public StationaryAimbotFullLaunch() {
        final Robot robot = Robot.getInstance();
        addCommands(
                new FullAim(),
                new WaitCommand(250),
                new ClearLaunch(true)
        );
    }
}
