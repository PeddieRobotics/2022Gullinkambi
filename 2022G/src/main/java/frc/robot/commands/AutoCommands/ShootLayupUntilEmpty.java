package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeCommands.CheckIfHopperEmpty;
import frc.robot.commands.ShootCommands.ShootLayup;

public class ShootLayupUntilEmpty extends ParallelRaceGroup{
    public ShootLayupUntilEmpty(){
        addCommands(
            new ShootLayup(),
            new CheckIfHopperEmpty(0.25)
        );
    }
}
