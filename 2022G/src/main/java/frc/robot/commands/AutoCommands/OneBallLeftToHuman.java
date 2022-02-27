package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SplitFFRamseteCommand;

public class OneBallLeftToHuman extends  SequentialCommandGroup{
    public OneBallLeftToHuman(SplitFFRamseteCommand oneBallLeftToHuman){
        addCommands(
            new ShootLayupForTime(2),
            oneBallLeftToHuman
        );
    }
}
