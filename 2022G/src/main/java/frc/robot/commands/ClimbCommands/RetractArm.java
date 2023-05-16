package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/** An example command that uses an example subsystem. */
public class RetractArm extends CommandBase {
  private Climber climber;

  public RetractArm() {
    climber = Climber.getInstance();
    addRequirements(climber);

  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.disablePIDController();
    climber.setClimberSolenoidBrake(true);
    climber.run(1); // Attempt to go past the limit sensor to make sure we reach it
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.run(0);
    climber.setClimberSolenoidBrake(false);
  }

  // Returns true when the command should end.
  // runs everytime execute runs
  @Override
  public boolean isFinished() {
    return climber.armSensorState();
  }
}
