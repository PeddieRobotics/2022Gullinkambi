package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.commands.ClimbCommands.*;
import frc.robot.utils.UpdateLogs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import frc.robot.utils.RobotMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class Climber extends SubsystemBase{
  private static Climber climber;
  private Solenoid arm;
  private Compressor compressor;

  private static UpdateLogs updateLogs = UpdateLogs.getInstance();

  private CANSparkMax armPrimary, armSecondary;

  public Climber() {
    armPrimary = new CANSparkMax(RobotMap.MOTOR_CLIMBER_PRIMARY, MotorType.kBrushless);
    armSecondary = new CANSparkMax(RobotMap.MOTOR_CLIMBER_SECONDARY, MotorType.kBrushless);
    armSecondary.follow(armPrimary);
  }
  
  public static Climber getInstance(){
    if(climber == null){
      climber = new Climber();
      climber.register();
    }
    return climber;
  }

  public void extend() {
    //arm.set(true);
  }

  public void retract() {
    //arm.set(false);
  }

  @Override
  public void periodic() {
    //updateLogs.updateClimberLogData();
  }

  public boolean getArmState(){
    return arm.get();
  }

  public double getCompressorPressure(){
    return compressor.getPressure();
  }
}
