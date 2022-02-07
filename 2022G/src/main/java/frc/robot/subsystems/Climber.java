package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.commands.ClimbCommands.*;
import frc.robot.utils.UpdateLogs;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class Climber extends SubsystemBase{
  private static Climber climber;
  private Solenoid arm;
  private Compressor compressor;

  private static UpdateLogs updateLogs = UpdateLogs.getInstance();

  public Climber() {
    arm = new Solenoid(PneumaticsModuleType.REVPH, 10);
    compressor = new Compressor(PneumaticsModuleType.REVPH);
  }
  
  public static Climber getInstance(){
    if(climber == null){
      climber = new Climber();
      climber.register();
    }
    return climber;
  }

  public void extend(){
    arm.set(true);
  }  

  public void retract(){
    arm.set(false);
  }  

  @Override
  public void periodic() {
    updateLogs.updateClimberLogData();
  }

  public boolean getArmState(){
    return arm.get();
  }

  public double getCompressorPressure(){
    return compressor.getPressure();
  }
}