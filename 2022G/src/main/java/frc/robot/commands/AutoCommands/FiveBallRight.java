package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommands.Drive;
import frc.robot.commands.DriveCommands.ResetOdometry;
import frc.robot.commands.DriveCommands.TurnToAngle;
import frc.robot.commands.IntakeCommands.AutoIntakeWithHopper;
import frc.robot.commands.IntakeCommands.StopIntake;
import frc.robot.commands.ShootCommands.SetFlywheelRPM;

public class FiveBallRight extends SequentialCommandGroup{ 

    public FiveBallRight(Pose2d initialPose, RamseteCommand part1, RamseteCommand part2, RamseteCommand part3, RamseteCommand part4){
        addCommands(
            new ResetOdometry(initialPose),
            new SetFlywheelRPM(2500),
            new ParallelCommandGroup(
                new AutoIntakeWithHopper(1, 1),
                part1
            ),
            new StopIntake(),
            new ShootWithLLUntilEmpty(0.3),
            new ParallelCommandGroup(
                new SequentialCommandGroup(new WaitCommand(0.5), new AutoIntakeWithHopper(1, 1)),
            part2),
            new ShootWithLLUntilEmpty(0.3),
            new AutoIntakeWithHopper(1, 0.7),
            part3,
            new WaitCommand(1),
            part4,
            new ShootWithLLUntilEmpty(0.3),
            new TurnToAngle(-20)
        );
    }
}
