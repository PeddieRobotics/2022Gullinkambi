package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class StopDrivetrain extends CommandBase{
    private Drivetrain mDrivetrain;

  public StopDrivetrain() {
    mDrivetrain = Drivetrain.getInstance();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDrivetrain.tankDriveVolts(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; // End immediately
  }
}
