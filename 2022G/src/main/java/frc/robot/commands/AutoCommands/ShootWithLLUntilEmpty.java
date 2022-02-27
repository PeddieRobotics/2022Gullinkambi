package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeCommands.CheckIfHopperEmpty;
import frc.robot.commands.ShootCommands.ShootWithLL;
import frc.robot.commands.ShootCommands.Target;
import frc.robot.subsystems.Hopper;

public class ShootWithLLUntilEmpty extends SequentialCommandGroup{

    public ShootWithLLUntilEmpty() {
        addCommands(    
            new ParallelRaceGroup(
                new Target(),
                new ShootWithLL(),
                new CheckIfHopperEmpty(0.25))
        );
    }
    
}
