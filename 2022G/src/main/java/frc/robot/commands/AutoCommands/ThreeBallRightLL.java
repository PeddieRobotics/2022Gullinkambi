package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SplitFFRamseteCommand;
import frc.robot.commands.DriveCommands.ResetOdometry;
import frc.robot.commands.IntakeCommands.AutoIntakeWithHopper;
import frc.robot.commands.IntakeCommands.StopIntake;
import frc.robot.commands.ShootCommands.SetFlywheelRPM;

public class ThreeBallRightLL extends SequentialCommandGroup{ 

    public ThreeBallRightLL(Pose2d initialPose, SplitFFRamseteCommand part1, SplitFFRamseteCommand part2){
        addCommands(
            new ResetOdometry(initialPose),
            new SetFlywheelRPM(2500),
            new ParallelCommandGroup(
                new AutoIntakeWithHopper(1.0, 0.7),
                part1
            ),
            new StopIntake(),
            new ShootWithLLUntilEmpty(),
            part2,
            new StopIntake(),
            new ShootWithLLForTime(5)
        );
    }
}
