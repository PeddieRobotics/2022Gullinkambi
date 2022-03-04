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
            new SetFlywheelRPM(Constants.FLYWHEEL_RPM_LAYUP),
            new ParallelCommandGroup(
                new AutoIntakeWithHopper(),
                part1
            ),
            new ShootWithLLForTime(2),
            new ParallelCommandGroup(
                part2,
                new AutoIntakeWithHopper()
            ),
            new WaitCommand(1),
            new SetFlywheelRPM(Constants.FLYWHEEL_RPM_LAYUP),
            part3,
            new TurnToAngle(-100),
            new ShootWithLLForTime(2),
            new ParallelCommandGroup(
                part4,
                new AutoIntakeWithHopper()
            ),
            new ShootWithLLForTime(5)
        );
    }
}
