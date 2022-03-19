package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommands.ResetOdometry;
import frc.robot.commands.DriveCommands.TurnToAngle;
import frc.robot.commands.IntakeCommands.AutoIntakeWithHopper;
import frc.robot.commands.IntakeCommands.StopIntake;
import frc.robot.commands.ShootCommands.SetFlywheelRPM;
import frc.robot.utils.Constants;

public class FourBallRightRude extends SequentialCommandGroup{ 

    public FourBallRightRude(Pose2d initialPose, RamseteCommand part1, RamseteCommand part2, RamseteCommand part3){
        addCommands(
            new ResetOdometry(initialPose),
            new SetFlywheelRPM(Constants.FLYWHEEL_RPM_LAYUP),
            new ParallelCommandGroup(
                new AutoIntakeWithHopper(0.7, 0.7),
                part1
            ),
            new ShootWithLLUntilEmpty(0.3),
            new AutoIntakeWithHopper(1.0, 0.7),
            new SetFlywheelRPM(Constants.FLYWHEEL_RPM_LAYUP),
            new TurnToAngle(155),
            part2,
            new StopIntake(),
            new ShootWithLLUntilEmpty(0.3),
            new SetFlywheelRPM(3000),
            new ParallelCommandGroup(
                new SequentialCommandGroup(new WaitCommand(0.5), new AutoIntakeWithHopper(1, 1)),
            part3),
            new StopIntake(),
            new TurnToAngle(-35),
            new ShootLowUntilEmpty(3000, 0.5)
        );
    }
}
