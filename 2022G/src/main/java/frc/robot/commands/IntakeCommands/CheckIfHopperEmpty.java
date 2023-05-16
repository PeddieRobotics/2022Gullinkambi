package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;

public class CheckIfHopperEmpty extends CommandBase {
  private Hopper hopper;
  private boolean hopperCleared;
  private double initialTime;
  private double waitTime;

  public CheckIfHopperEmpty(double delay) {
    hopper = Hopper.getInstance();
    hopperCleared = false;
    waitTime = delay;
  }

  @Override
  public void initialize() {
    hopperCleared = false;
  }

  @Override
  public void execute() {
    // Repeatedly check whether both sensors are clear
    if(!hopper.sensesBallTop() && !hopper.sensesBallBottom()){
        if(!hopperCleared){
          initialTime = Timer.getFPGATimestamp();
          hopperCleared = true;
        }
    }
    else{
      hopperCleared = false;
    }

  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hopperCleared && (Timer.getFPGATimestamp()-initialTime > waitTime);
  }

}