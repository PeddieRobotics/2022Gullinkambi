package frc.robot.commands.DriveCommands;

import java.sql.Driver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ResetOdometry extends CommandBase{
    private Drivetrain drivetrain;
    private Pose2d newPose;
    public ResetOdometry(Pose2d newPose){
        drivetrain = Drivetrain.getInstance();
        this.newPose = newPose;
    }

    @Override
    public void initialize(){
        drivetrain.resetPose(newPose, drivetrain.getHeadingAsRotation2d());
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
