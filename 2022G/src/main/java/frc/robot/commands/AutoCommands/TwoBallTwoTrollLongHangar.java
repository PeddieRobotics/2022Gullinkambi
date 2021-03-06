package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommands.ResetOdometry;
import frc.robot.commands.DriveCommands.StopDrivetrain;
import frc.robot.commands.DriveCommands.TurnToAngle;
import frc.robot.commands.IntakeCommands.AutoIntakeWithHopper;
import frc.robot.commands.IntakeCommands.StopIntake;
import frc.robot.commands.IntakeCommands.UnjamIntake;
import frc.robot.commands.ShootCommands.SetFlywheelRPM;
import frc.robot.utils.Constants;

public class TwoBallTwoTrollLongHangar extends SequentialCommandGroup{
    public TwoBallTwoTrollLongHangar(Pose2d initialPose, RamseteCommand part1, RamseteCommand part2, RamseteCommand part3){
        addCommands(
            new ResetOdometry(initialPose),
            new AutoWaitFromDashboard("Troll Auto Start Delay"),
            new SetFlywheelRPM(Constants.FLYWHEEL_RPM_LAYUP),
            new ParallelCommandGroup(
                new AutoIntakeWithHopper(1.0, 0.7),
                part1
            ),
            new ShootWithLLUntilEmpty(0.3),
            new SetFlywheelRPM(0),
            new AutoWaitFromDashboard("Troll Auto Post-Shoot Delay"),
            new TurnToAngle(60),
            part2,
            new TurnToAngle(175),
            part3,
            new StopDrivetrain(),
            new StopIntake(),
            new TurnToAngle(140),
            new ParallelRaceGroup(new UnjamIntake(0.6, false), new WaitCommand(3))
        );
    }
}