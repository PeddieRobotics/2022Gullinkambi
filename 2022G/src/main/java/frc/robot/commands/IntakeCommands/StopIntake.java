package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class StopIntake extends CommandBase {

  private Intake mIntake;

  public StopIntake() {
    mIntake = Intake.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mIntake.stopIntake();
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
