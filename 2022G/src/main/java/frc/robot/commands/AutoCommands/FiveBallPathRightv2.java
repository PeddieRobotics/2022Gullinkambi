package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SplitFFRamseteCommand;
import frc.robot.commands.DriveCommands.ResetOdometry;
import frc.robot.commands.DriveCommands.TurnToAngle;
import frc.robot.commands.IntakeCommands.AutoIntakeWithHopper;
import frc.robot.commands.IntakeCommands.StopIntake;
import frc.robot.commands.ShootCommands.SetFlywheelRPM;
import frc.robot.commands.ShootCommands.ShootWithLL;
import frc.robot.utils.Constants;

public class FiveBallPathRightv2 extends SequentialCommandGroup{ 

    public FiveBallPathRightv2(Pose2d initialPose, SplitFFRamseteCommand part1, SplitFFRamseteCommand part2, SplitFFRamseteCommand part3, SplitFFRamseteCommand part4){
        addCommands(
            new ResetOdometry(initialPose),
            new SetFlywheelRPM(Constants.FLYWHEEL_RPM_LAYUP),
            new ParallelCommandGroup(
                new AutoIntakeWithHopper(),
                part1
            ),
            new ShootWithLLForTime(1.5),
            new SetFlywheelRPM(Constants.FLYWHEEL_RPM_LAYUP),
            new AutoIntakeWithHopper(),
            new TurnToAngle(165),
            part2,
            new TurnToAngle(-140),
            new ShootWithLLForTime(1),
            new AutoIntakeWithHopper(),
            part3,
            new WaitCommand(1),
            part4,
            new StopIntake(),
            new ShootWithLLForTime(5)
        );
    }
}
