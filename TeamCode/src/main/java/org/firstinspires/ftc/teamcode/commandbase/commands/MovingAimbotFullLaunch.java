package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
public class MovingAimbotFullLaunch extends SequentialCommandGroup {
    public MovingAimbotFullLaunch() {
        addCommands(
                new MovingAim(),
                new WaitCommand(250),
                new ClearLaunch(true)
        );
    }
}
