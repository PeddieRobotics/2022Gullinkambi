package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommands.ResetOdometry;
import frc.robot.commands.DriveCommands.StopDrivetrain;
import frc.robot.commands.DriveCommands.TurnToAngle;
import frc.robot.commands.IntakeCommands.AutoIntakeWithHopper;
import frc.robot.commands.IntakeCommands.UnjamIntake;
import frc.robot.commands.IntakeCommands.UnjamIntakeForTime;
import frc.robot.commands.ShootCommands.SetFlywheelRPM;
import frc.robot.utils.Constants;

public class TwoBallOneTrollFender extends SequentialCommandGroup{
    public TwoBallOneTrollFender(Pose2d initialPose, RamseteCommand part1, RamseteCommand part2){
        addCommands(
            new ResetOdometry(initialPose),
            new SetFlywheelRPM(Constants.FLYWHEEL_RPM_LAYUP),
            new ParallelCommandGroup(
                 new AutoIntakeWithHopper(1.0, 0.7),
                 part1),
            new ShootWithLLUntilEmpty(0.5),
            new SetFlywheelRPM(0),
            new TurnToAngle(60),
            part2,
            //new TurnToAngle(-65),
            new StopDrivetrain(),
            new WaitCommand(3),
            new UnjamIntake(0.4, false)
            //new TurnToAngle(28)
        );
    }
}
