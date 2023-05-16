package frc.robot.commands.LightCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;

/** An example command that uses an example subsystem. */
public class FlashLights extends CommandBase {
  private Lights light;
  double initialTime, previousFlashTime, totalTime, rate;

  public FlashLights(double seconds, double flashRate) {
    light = Lights.getInstance();
    initialTime = 0.0;
    previousFlashTime = 0.0;
    totalTime = seconds;
    rate = flashRate;

    addRequirements(light);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      initialTime = Timer.getFPGATimestamp();
      SmartDashboard.putNumber("light initial time", initialTime);
      previousFlashTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("light periodic running", true);

    if(Timer.getFPGATimestamp()-previousFlashTime > rate){
        if(light.isOn()){
            light.off();
        }
        else{
            light.on();
        }
        previousFlashTime = Timer.getFPGATimestamp();
        SmartDashboard.putNumber("light prev time", previousFlashTime);

    }

  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      light.off();
    }
  

  // Returns true when the command should end.
  //runs everytime execute runs
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp()-initialTime) > totalTime;
  }
}