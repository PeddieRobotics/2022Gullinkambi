package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.IntakeCommands.CheckIfHopperEmpty;
import frc.robot.commands.ShootCommands.ShootLayup;

public class ShootLayupUntilEmpty extends ParallelRaceGroup{
    public ShootLayupUntilEmpty(double delay){
        addCommands(
            new ShootLayup(true),
            new CheckIfHopperEmpty(delay)
        );
    }
}
