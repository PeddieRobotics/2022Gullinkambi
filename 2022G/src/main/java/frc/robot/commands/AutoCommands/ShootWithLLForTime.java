package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShootCommands.ShootWithLL;
import frc.robot.commands.ShootCommands.Target;

public class ShootWithLLForTime extends SequentialCommandGroup{

    public ShootWithLLForTime(int x) {
        addCommands(
            new Target(),
            new ParallelRaceGroup(
                new ShootWithLL(),
                new WaitCommand(x)
            )
        );
    }
    
}
