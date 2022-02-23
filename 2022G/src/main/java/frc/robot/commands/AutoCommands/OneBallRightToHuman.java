package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SplitFFRamseteCommand;

public class OneBallRightToHuman extends  SequentialCommandGroup{
    public OneBallRightToHuman(SplitFFRamseteCommand oneBallRightToHuman){
        addCommands(
            new ShootForTimed(2),
            oneBallRightToHuman
        );
    }
}
