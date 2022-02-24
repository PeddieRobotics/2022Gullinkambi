package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.utils.Constants;

/** An example command that uses an example subsystem. */
public class ExtendArm extends CommandBase {
  private Climber climber;

  public ExtendArm() {
    climber = Climber.getInstance();
    addRequirements(climber);
  }

   // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.enablePIDController();
    climber.setClimberSolenoidBrake(false);
    climber.moveToPosition(Constants.CLIMBER_TOP_ENCODER_POSITION);
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
    return true;
  }
}
