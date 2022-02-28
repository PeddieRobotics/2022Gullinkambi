package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SplitFFRamseteCommand;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commands.DriveCommands.ResetOdometry;
import frc.robot.utils.Constants;
import frc.robot.commands.ShootCommands.SetFlywheelRPM;

public class OneBallLeftToHuman extends  SequentialCommandGroup{
    public OneBallLeftToHuman(Pose2d initialPose, SplitFFRamseteCommand oneBallLeftToHuman){
        addCommands(
            new ResetOdometry(initialPose),
            new SetFlywheelRPM(Constants.FLYWHEEL_RPM_LAYUP),
            new ShootLayupForTime(2),
            oneBallLeftToHuman
        );
    }
}
