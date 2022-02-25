package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SplitFFRamseteCommand;
import frc.robot.commands.IntakeCommands.AutoIntakeWithHopper;
import frc.robot.commands.IntakeCommands.RunIntake;
import frc.robot.commands.IntakeCommands.StopIntake;
import frc.robot.commands.ShootCommands.ShootWithLL;
import frc.robot.commands.ShootCommands.Target;
import frc.robot.commands.AutoCommands.ShootWithLLForTime;

public class TwoBallLeftUpShoot extends  SequentialCommandGroup{
    public TwoBallLeftUpShoot(SplitFFRamseteCommand twoBallLeftUpShoot){
        addCommands(
            new ParallelCommandGroup(
                new AutoIntakeWithHopper(),
                twoBallLeftUpShoot
            ),
            new StopIntake(),
            new ShootWithLLForTime(4)
        );
    }
}
