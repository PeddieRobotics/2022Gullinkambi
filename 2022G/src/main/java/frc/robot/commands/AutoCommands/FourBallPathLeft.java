package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SplitFFRamseteCommand;
import frc.robot.commands.DriveCommands.LLDriveToTarget;
import frc.robot.commands.DriveCommands.ResetOdometry;
import frc.robot.commands.DriveCommands.TurnByAngle;
import frc.robot.commands.IntakeCommands.AutoIntakeWithHopper;
import frc.robot.commands.IntakeCommands.StopIntake;
import frc.robot.commands.ShootCommands.SetFlywheelRPM;
import frc.robot.utils.Constants;

public class FourBallPathLeft extends SequentialCommandGroup{ 

    public FourBallPathLeft(Pose2d initialPose, SplitFFRamseteCommand part1, SplitFFRamseteCommand part2, SplitFFRamseteCommand part3){
        addCommands(
            new ResetOdometry(initialPose),
            new SetFlywheelRPM(Constants.FLYWHEEL_RPM_LAYUP),
            new ParallelCommandGroup(
                new AutoIntakeWithHopper(),
                part1
            ),
            new StopIntake(),
            new ShootWithLLUntilEmpty(),
            new TurnByAngle(93),
            new ParallelCommandGroup(
                part2,
                new SequentialCommandGroup(new WaitCommand(1.5), new AutoIntakeWithHopper())
            ),
            new WaitCommand(1.5),
            new StopIntake(),
            new SetFlywheelRPM(Constants.FLYWHEEL_RPM_LAYUP),
            part3,
            new TurnByAngle(-93),
            new ShootWithLLUntilEmpty()
        );
    }
}
