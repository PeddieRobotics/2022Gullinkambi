package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommands.ResetOdometry;
import frc.robot.commands.DriveCommands.TurnToAngle;
import frc.robot.commands.IntakeCommands.AutoIntakeWithHopper;
import frc.robot.commands.ShootCommands.SetFlywheelRPM;

public class FiveBallPathRightv2 extends SequentialCommandGroup{ 

    public FiveBallPathRightv2(Pose2d initialPose, RamseteCommand part1, RamseteCommand part2, RamseteCommand part3, RamseteCommand part4){
        addCommands(
            new ResetOdometry(initialPose),
            new SetFlywheelRPM(2650),
            new ParallelCommandGroup(
                new AutoIntakeWithHopper(1, 1),
                part1
            ),
            new ShootWithLLUntilEmpty(0.3),
            new AutoIntakeWithHopper(1, 1),
            part2,
            new TurnToAngle(-142),
            new ShootWithLLUntilEmpty(0.3),
            new AutoIntakeWithHopper(1, 0.7),
            part3,
            new WaitCommand(1),
            part4,
            new ShootWithLLForTime(5)
        );
    }
}
