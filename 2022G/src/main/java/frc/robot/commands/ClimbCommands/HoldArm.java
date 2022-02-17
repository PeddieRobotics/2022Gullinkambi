package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/** An example command that uses an example subsystem. */
public class HoldArm extends CommandBase {
  private Climber climber;

  public HoldArm() {
    climber = Climber.getInstance();
    addRequirements(climber);

  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.moveToPosition(0.0); // Hold at encoder position 0
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
  // runs everytime execute runs
  @Override
  public boolean isFinished() {
    return false;
  }
}
