package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Climber;

/** An example command that uses an example subsystem. */
public class RetractRightArm extends CommandBase {
  private Climber climber;

  public RetractRightArm() {
    climber = Climber.getInstance();

  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.disableRightPIDController();
    climber.setRightArmSolenoidBrake(true);
    climber.setRightArmMotorSpeed(1); // Attempt to go past the limit sensor to make sure we reach it
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setRightArmMotorSpeed(0);
    climber.setRightArmSolenoidBrake(false);
  }

  // Returns true when the command should end.
  // runs everytime execute runs
  @Override
  public boolean isFinished() {
    return climber.rightArmSensorState();
  }
}
