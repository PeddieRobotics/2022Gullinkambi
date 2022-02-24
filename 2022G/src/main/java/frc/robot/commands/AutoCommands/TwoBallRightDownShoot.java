package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SplitFFRamseteCommand;
import frc.robot.commands.IntakeCommands.RunIntake;
import frc.robot.commands.IntakeCommands.StopIntake;
import frc.robot.commands.ShootCommands.ShootWithLL;
import frc.robot.commands.ShootCommands.Target;

public class TwoBallRightDownShoot extends  SequentialCommandGroup{
    public TwoBallRightDownShoot(SplitFFRamseteCommand twoBallRightDownShoot){
        addCommands(
            new ParallelCommandGroup(
                new RunIntake(),
                twoBallRightDownShoot
            ),
            new StopIntake(),
            new ShootWithLLForTime(4)
        );
    }
}
