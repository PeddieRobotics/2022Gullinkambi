package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.commands.ClimbCommands.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climber extends SubsystemBase {
  private static Climber climber;
  private Solenoid arm;

  public Climber() {
    arm = new Solenoid(PneumaticsModuleType.CTREPCM, 10);
  }

  public static Climber getInstance() {
    if (climber == null) {
      climber = new Climber();
      climber.register();
    }
    return climber;
  }

  public void extend() {
    arm.set(true);
  }

  public void armSet(boolean on){
    arm.set(on);
  }
  public void getArm(){
    arm.get();
  }

  public void retract() {
    arm.set(false);
  }

  @Override
  public void periodic() {
  }

  public void putSmartDashboardOverrides(){
    SmartDashboard.putBoolean("OR: Climber Extend", false);
  }

  public void updateFromDashboard(){
    armSet(SmartDashboard.getBoolean("OR: Climber Extend", false));
  }
}