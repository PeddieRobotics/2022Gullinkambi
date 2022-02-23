package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShootCommands.ShootLayup;

public class ShootForTimed extends ParallelRaceGroup{
    public ShootForTimed(int x){
        addCommands(
            new ShootLayup(),
            new WaitCommand(x)
        );
    }
}
