package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Climber;

/** An example command that uses an example subsystem. */
public class InitializeLeftArm extends CommandBase {
  private Climber climber;
  boolean firstMovementDownward;

  public InitializeLeftArm() {
    climber = Climber.getInstance();
    firstMovementDownward = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setLeftArmSolenoidBrake(true);
    climber.setLeftArmMotorSpeed(-0.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!climber.leftArmSensorState()){
      climber.setLeftArmMotorSpeed(0.25);
      firstMovementDownward = true;
     }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setLeftArmEncoderPosition(0); // Call this retracted position the new "zero"
    climber.setLeftArmMotorSpeed(0);
    climber.setLeftArmSolenoidBrake(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (climber.leftArmSensorState() && firstMovementDownward);
  }
}