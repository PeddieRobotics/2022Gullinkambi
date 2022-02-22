package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SplitFFRamseteCommand;

public class OneBallLeftUp extends  SequentialCommandGroup{
    public OneBallLeftUp(SplitFFRamseteCommand oneBallLeftUp){
        addCommands(
            new ShootForTimed(3),
            oneBallLeftUp
        );
    }
}
