package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SplitFFRamseteCommand;
import frc.robot.commands.IntakeCommands.RunIntake;
import frc.robot.commands.IntakeCommands.StopIntake;

public class TwoBallLeftUpShoot extends  SequentialCommandGroup{
    public TwoBallLeftUpShoot(SplitFFRamseteCommand twoBallLeftUpShoot){
        addCommands(
            new ParallelCommandGroup(
                new RunIntake(),
                twoBallLeftUpShoot
            ),
            new StopIntake(),
            new ShootForTimed(4)
        );
    }
}
