package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SplitFFRamseteCommand;
import frc.robot.commands.DriveCommands.LLDriveToTarget;
import frc.robot.commands.DriveCommands.ResetOdometry;
import frc.robot.commands.DriveCommands.TurnByAngle;
import frc.robot.commands.IntakeCommands.AutoIntakeWithHopper;
import frc.robot.commands.IntakeCommands.StopIntake;
import frc.robot.commands.ShootCommands.SetFlywheelRPM;
import frc.robot.utils.Constants;

public class FourBallPathRightNoLayup extends SequentialCommandGroup{ 

    public FourBallPathRightNoLayup(Pose2d initialPose, SplitFFRamseteCommand part1, SplitFFRamseteCommand part2){
        addCommands(
            new ResetOdometry(initialPose),
            new SetFlywheelRPM(Constants.FLYWHEEL_RPM_LAYUP),
            new ParallelCommandGroup(
                new AutoIntakeWithHopper(),
                part1
            ),
            new ShootWithLLUntilEmpty(),
            new TurnByAngle(97),
            part2,
            new TurnByAngle(-20),
            new StopIntake(),
            new SetFlywheelRPM(Constants.FLYWHEEL_RPM_LAYUP),
            new LLDriveToTarget(0.35, 0.0),
            new ShootWithLLUntilEmpty()
        );
    }
}
