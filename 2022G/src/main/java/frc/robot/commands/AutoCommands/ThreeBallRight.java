package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SplitFFRamseteCommand;
import frc.robot.commands.IntakeCommands.RunIntake;
import frc.robot.commands.IntakeCommands.StopIntake;
import frc.robot.commands.ShootCommands.ShootLayup;
import frc.robot.commands.ShootCommands.ShootWithLL;
import frc.robot.commands.ShootCommands.Target;

public class ThreeBallRight extends SequentialCommandGroup{
    public ThreeBallRight(SplitFFRamseteCommand part1, SplitFFRamseteCommand part2){
        addCommands(
            part1,
            new ShootForTimed(3),
            part2,
            new ShootWithLLForTime(3)
        );
    }
}
