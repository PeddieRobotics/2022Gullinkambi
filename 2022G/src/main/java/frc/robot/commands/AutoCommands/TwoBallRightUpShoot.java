package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SplitFFRamseteCommand;
import frc.robot.commands.IntakeCommands.RunIntake;
import frc.robot.commands.IntakeCommands.StopIntake;
import frc.robot.commands.ShootCommands.ShootWithLL;
import frc.robot.commands.ShootCommands.Target;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commands.DriveCommands.ResetOdometry;
import frc.robot.utils.Constants;
import frc.robot.commands.ShootCommands.SetFlywheelRPM;

public class TwoBallRightUpShoot extends  SequentialCommandGroup{
    public TwoBallRightUpShoot(Pose2d initialPose, SplitFFRamseteCommand twoBallRightUpShoot){
        addCommands(
            new ResetOdometry(initialPose),
            new SetFlywheelRPM(Constants.FLYWHEEL_RPM_LAYUP),
            new ParallelCommandGroup(
                new RunIntake(),
                twoBallRightUpShoot
            ),
            new StopIntake(),
            new ParallelCommandGroup(
                new Target(),
                new ShootWithLL()
            )
        );
    }
}
