package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.utils.Constants;

public class FlywheelEdgeDetection extends CommandBase {
  private Flywheel flywheel;
  private int edgesToDetect, edgesRemaining;
  public boolean armed;

  public FlywheelEdgeDetection(int edges) {
    flywheel = Flywheel.getInstance();
    edgesToDetect = edges;
    armed = false;
  }

  @Override
  public void initialize() {
    edgesRemaining = edgesToDetect;
  }

  @Override
  public void execute() {
    // Arm and trigger algorithm for rising edge detection
    if(flywheel.getFlywheelVelocity() > 0.95*flywheel.getFlywheelSetpoint()){
      armed = true;
    } 
    if(armed){
      if(flywheel.getFlywheelVelocity() > 15){
          edgesRemaining--;
          armed = false;
      }
    }

    // // Arm and trigger algorithm for falling edge detection
    // if(flywheel.getFlywheelVelocity() > 0.95*flywheel.getFlywheelSetpoint()){
    //     armed = true;
    // } 
    // if(armed){
    //     if(flywheel.getFlywheelVelocity() < 0.9*flywheel.getFlywheelSetpoint()){
    //         edgesRemaining--;
    //         armed = false;
    //     }
    // }

  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return edgesRemaining == 0;
  }

}