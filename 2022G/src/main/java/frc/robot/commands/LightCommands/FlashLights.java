package frc.robot.commands.LightCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;

/** An example command that uses an example subsystem. */
public class FlashLights extends CommandBase {
  private Lights light;
  double initialTime, previousFlashTime, totalTime;

  public FlashLights(double seconds) {
    light = light.getInstance();
    initialTime = 0.0;
    previousFlashTime = 0.0;
    totalTime = seconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      initialTime = Timer.getFPGATimestamp();
      light.on();
      previousFlashTime = Timer.getFPGATimestamp();
      light.setLightsFlashing(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Timer.getFPGATimestamp()-previousFlashTime > 0.5){
        if(light.isOn()){
            light.off();
        }
        else{
            light.on();
        }
        previousFlashTime = Timer.getFPGATimestamp();

    }

  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      light.on();
      light.setLightsFlashing(false);
    }
  

  // Returns true when the command should end.
  //runs everytime execute runs
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp()-initialTime) > totalTime;
  }
}