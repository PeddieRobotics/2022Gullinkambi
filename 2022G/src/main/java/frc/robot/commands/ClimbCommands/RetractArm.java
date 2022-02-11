package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/** An example command that uses an example subsystem. */
public class RetractArm extends CommandBase {
  private Climber climber;

  public RetractArm() {
    climber = Climber.getInstance();
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.retract();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  // runs everytime execute runs
  @Override
  public boolean isFinished() {
    return true;
  }
}
