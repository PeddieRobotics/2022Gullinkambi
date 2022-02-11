package frc.robot.commands.LightCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;

/** An example command that uses an example subsystem. */
public class TurnOn extends CommandBase {
  private Lights light;
  public TurnOn() {
    light = light.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    light.on();
    }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    }
  

  // Returns true when the command should end.
  //runs everytime execute runs
  @Override
  public boolean isFinished() {
    return true;
  }
}