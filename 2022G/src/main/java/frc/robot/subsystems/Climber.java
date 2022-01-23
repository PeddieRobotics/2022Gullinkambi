// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.RobotMap;

public class Climber extends SubsystemBase {
  private static Climber climber;

  private CANSparkMax leftArmMotor, rightArmMotor;
  private Solenoid leftArmSolenoid, rightArmSolenoid, hookSolenoid;

  public Climber() {
    leftArmMotor = new CANSparkMax(RobotMap.MOTOR_CLIMBER_LEFT, MotorType.kBrushless);
    rightArmMotor = new CANSparkMax(RobotMap.MOTOR_CLIMBER_RIGHT, MotorType.kBrushless);

    leftArmSolenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.SOLENOID_CLIMBER_LEFT);
    rightArmSolenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.SOLENOID_CLIMBER_RIGHT);

    hookSolenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.SOLENOID_CLIMBER_HOOK);

  }

  public static Climber getInstance() {
    if (climber == null) {
      climber = new Climber();
    }

    return climber;
  }

  public boolean isClimberTilted() {
    return (leftArmSolenoid.get() && rightArmSolenoid.get());
  }

  public boolean isHookDeployed(){
    return hookSolenoid.get();
  }

  public void setClimberTilt(boolean state) {
    leftArmSolenoid.set(state);
    rightArmSolenoid.set(state);
  }

  public void setClimberHook(boolean state) {
    hookSolenoid.set(state);
  }

  public void setClimberSpeed(double speed){
    leftArmMotor.set(speed);
    rightArmMotor.set(speed);
  }

  public void putSmartDashboardOverrides(){
    SmartDashboard.putNumber("OR: Climber speed", 0.0);
    SmartDashboard.putBoolean("OR: Climber tilt", false);
    SmartDashboard.putBoolean("OR: Climber hook", false);

}
}

