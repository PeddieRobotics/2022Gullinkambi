package frc.robot.commands.LightCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Lights;

/** An example command that uses an example subsystem. */
public class SignalWithLights extends CommandBase {
  private Lights lights;
  private Hopper hopper;

  public SignalWithLights() {
    lights = Lights.getInstance();
    hopper = Hopper.getInstance();

   addRequirements(lights); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is sc{heduled.
  @Override
  public void execute() {
    if(hopper.sensesBallBottom() && hopper.sensesBallTop()){
        lights.on();
    }
    else{
        lights.off();
    }

  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      lights.off();
    }
  

  // Returns true when the command should end.
  //runs everytime execute runs
  @Override
  public boolean isFinished() {
    return false;
  }
}