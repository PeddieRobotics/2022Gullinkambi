package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommands.CheckIfHopperEmpty;
import frc.robot.commands.ShootCommands.ShootLow;

public class ShootLowUntilEmpty extends SequentialCommandGroup{

    public ShootLowUntilEmpty(double delay) {
        addCommands(    
            new ParallelRaceGroup(
                new ShootLow(),
                new CheckIfHopperEmpty(delay))
        );
    }
    
}
