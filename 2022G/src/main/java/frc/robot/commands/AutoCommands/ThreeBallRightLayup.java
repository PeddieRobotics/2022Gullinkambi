package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SplitFFRamseteCommand;
import frc.robot.commands.DriveCommands.ResetOdometry;
import frc.robot.commands.IntakeCommands.AutoIntakeWithHopper;
import frc.robot.commands.ShootCommands.SetFlywheelRPM;
import frc.robot.utils.Constants;

public class ThreeBallRightLayup extends SequentialCommandGroup{
    public ThreeBallRightLayup(Pose2d initialPose, SplitFFRamseteCommand part1, SplitFFRamseteCommand part2){
        addCommands(
            new ResetOdometry(initialPose),
            new SetFlywheelRPM(Constants.FLYWHEEL_RPM_LAYUP),
            new AutoIntakeWithHopper(1.0, 0.7),
            part1,
            new ShootLayupUntilEmpty(),
            part2,
            new ShootLayupForTime(5)
        );
    }
}
