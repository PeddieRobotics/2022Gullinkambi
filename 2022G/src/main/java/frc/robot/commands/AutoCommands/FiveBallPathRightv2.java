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

public class FiveBallPathRightv2 extends SequentialCommandGroup{ 

    public FiveBallPathRightv2(Pose2d initialPose, SplitFFRamseteCommand part1, SplitFFRamseteCommand part2, SplitFFRamseteCommand part3, SplitFFRamseteCommand part4){
        addCommands(
            new ResetOdometry(initialPose),
            new SetFlywheelRPM(2500),
            new ParallelCommandGroup(
                new AutoIntakeWithHopper(0.7, 0.7),
                part1
            ),
            new ShootWithLLUntilEmpty(),
            new AutoIntakeWithHopper(0.7, 0.7),
            new SetFlywheelRPM(2500),
            part2,
            new TurnToAngle(-145),
            new ShootWithLLUntilEmpty(),
            new SetFlywheelRPM(2500),
            new AutoIntakeWithHopper(1.0, 0.7),
            part3,
            new WaitCommand(1.5),
            part4,
            new StopIntake(),
            new ShootWithLLForTime(5)
        );
    }
}
