package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Climber;

/** An example command that uses an example subsystem. */
public class InitializeArm extends CommandBase {
  private Climber climber;

  public InitializeArm() {
    climber = Climber.getInstance();
    addRequirements(climber);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      climber.run(0.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setEncoderPosition(0); // Call this retracted position the new "zero"
    climber.run(0);
    climber.moveToPosition(0); // Hold at encoder position 0
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.armSensorState();
  }
}