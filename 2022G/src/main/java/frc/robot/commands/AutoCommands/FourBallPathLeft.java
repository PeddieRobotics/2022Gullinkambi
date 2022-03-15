package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommands.ResetOdometry;
import frc.robot.commands.DriveCommands.TurnToAngle;
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
                new AutoIntakeWithHopper(1.0, 0.7),
                part1
            ),
            new StopIntake(),
            new ShootWithLLUntilEmpty(0.3),
            new TurnToAngle(-135),
            new ParallelCommandGroup(
                part2,
                new SequentialCommandGroup(new WaitCommand(1.3), new AutoIntakeWithHopper(1.0, 0.7))
            ),
            new WaitCommand(1),
            new SetFlywheelRPM(Constants.FLYWHEEL_RPM_LAYUP),
            part3,
            new StopIntake(),
            new TurnToAngle(150),
            new ShootWithLLForTime(5)
        );
    }
}
