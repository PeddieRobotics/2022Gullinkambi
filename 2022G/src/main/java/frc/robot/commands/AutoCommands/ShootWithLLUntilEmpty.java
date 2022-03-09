package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeCommands.CheckIfHopperEmpty;
import frc.robot.commands.ShootCommands.BlankCommand;
import frc.robot.commands.ShootCommands.ShootWithLL;
import frc.robot.commands.ShootCommands.Target;
import frc.robot.subsystems.Drivetrain;

public class ShootWithLLUntilEmpty extends SequentialCommandGroup{

    public ShootWithLLUntilEmpty() {
        addCommands(    
            new ParallelRaceGroup(
                new SequentialCommandGroup(new Target(), new ConditionalCommand(new ShootWithLL(true), new BlankCommand(), Drivetrain.getInstance()::isLockedOnTarget)),
                new CheckIfHopperEmpty(0.1))
        );
    }
    
}
