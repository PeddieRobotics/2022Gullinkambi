package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SplitFFRamseteCommand;
import frc.robot.commands.IntakeCommands.AutoIntakeWithHopper;
import frc.robot.commands.IntakeCommands.RunIntake;
import frc.robot.commands.IntakeCommands.StopIntake;
import frc.robot.commands.ShootCommands.RevUpFlywheel;
import frc.robot.commands.ShootCommands.SetFlywheelRPM;
import frc.robot.commands.ShootCommands.ShootLayup;
import frc.robot.commands.ShootCommands.ShootWithLL;
import frc.robot.commands.ShootCommands.Target;
import frc.robot.utils.Constants;

public class FourBallPathRight extends  SequentialCommandGroup{
    public FourBallPathRight(SplitFFRamseteCommand part1, SplitFFRamseteCommand part2){
        addCommands(
            new SetFlywheelRPM(Constants.FLYWHEEL_RPM_LAYUP),
            new ParallelCommandGroup(
                new AutoIntakeWithHopper(),//
                part1
            ),
            new StopIntake(),
            new ShootForTimed(1.75),
            new SetFlywheelRPM(Constants.FLYWHEEL_RPM_LAYUP),
            new ParallelCommandGroup(
                part2,
                new AutoIntakeWithHopper()
            ),
            new StopIntake(),
            new ShootWithLLForTime(2)
        );
    }
}
