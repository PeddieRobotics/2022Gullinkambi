package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SplitFFRamseteCommand;
import frc.robot.commands.DriveCommands.ResetOdometry;
import frc.robot.commands.DriveCommands.TurnToAngle;
import frc.robot.commands.IntakeCommands.AutoIntakeWithHopper;
import frc.robot.commands.IntakeCommands.StopIntake;
import frc.robot.commands.ShootCommands.SetFlywheelRPM;
import frc.robot.utils.Constants;

public class FiveBallPathRight extends SequentialCommandGroup{ 

    public FiveBallPathRight(Pose2d initialPose, SplitFFRamseteCommand part1, SplitFFRamseteCommand part2, SplitFFRamseteCommand part3, SplitFFRamseteCommand part4){
        addCommands(
            new ResetOdometry(initialPose),
            new SetFlywheelRPM(2500),
            new ParallelCommandGroup(
                new AutoIntakeWithHopper(1.0, 0.7),
                part1
            ),
            new ShootWithLLUntilEmpty(),
            new ParallelCommandGroup(
                part2,
                new AutoIntakeWithHopper(1.0, 0.7)
            ),
            new WaitCommand(1),
            new SetFlywheelRPM(2500),
                part3,
            new ShootWithLLUntilEmpty(),
            new ParallelCommandGroup(
                part4,
                new AutoIntakeWithHopper(1.0, 0.7)
            ),
            new ShootWithLLForTime(5)
        );
    }
}
