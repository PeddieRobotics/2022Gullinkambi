package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/** An example command that uses an example subsystem. */
public class InitializeArm extends CommandBase {
  private Climber climber;
  boolean firstMovementDownward;

  public InitializeArm() {
    climber = Climber.getInstance();
    addRequirements(climber);
    firstMovementDownward = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setClimberSolenoidBrake(true);
    climber.run(-0.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!climber.armSensorState()){
      climber.run(0.25);
      firstMovementDownward = true;
     }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setEncoderPosition(0); // Call this retracted position the new "zero"
    climber.run(0);
    climber.setClimberSolenoidBrake(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (climber.armSensorState() && firstMovementDownward);
  }
}