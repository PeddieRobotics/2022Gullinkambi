package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommands.ResetOdometry;
import frc.robot.commands.DriveCommands.TurnToAngle;
import frc.robot.commands.IntakeCommands.AutoIntakeWithHopper;
import frc.robot.commands.IntakeCommands.StopIntake;
import frc.robot.commands.ShootCommands.SetFlywheelRPM;

public class FiveBallPathRight extends SequentialCommandGroup{ 

    public FiveBallPathRight(Pose2d initialPose, RamseteCommand part1, RamseteCommand part2, RamseteCommand part3, RamseteCommand part4){
        addCommands(
            new ResetOdometry(initialPose),
            new SetFlywheelRPM(2500),
            new ParallelCommandGroup(
                new AutoIntakeWithHopper(1.0, 0.7),
                part1
            ),
            new ShootWithLLUntilEmpty(0.3),
            new ParallelCommandGroup(
                part2,
                new AutoIntakeWithHopper(0.7, 0.7)
            ),
            new WaitCommand(1.5),
            new SetFlywheelRPM(2500),
                part3,
            new TurnToAngle(-100),
            new ShootWithLLUntilEmpty(0.3),
            new ParallelCommandGroup(
                new AutoIntakeWithHopper(0.7, 0.7),
                part4
            ),
            new StopIntake(),
            new ShootWithLLForTime(5)
        );
    }
}
