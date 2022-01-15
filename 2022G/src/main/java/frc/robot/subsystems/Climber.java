// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.utils.RobotMap;

public class Climber extends SubsystemBase {
    private static Climber climber;

  private Solenoid leftArmSolenoid, rightArmSolenoid;

  public Climber() {
    leftArmSolenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.SOLENOID_CLIMBER_LEFT);
    rightArmSolenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.SOLENOID_CLIMBER_RIGHT);
  }

  public static Climber getInstance() {
    if (climber == null) {
      climber = new Climber();
    }

    return climber;
  }

  public boolean isClimberUp() {
    return (leftArmSolenoid.get() && rightArmSolenoid.get());
  }

  public void raiseClimber() {
    leftArmSolenoid.set(true);
    rightArmSolenoid.set(true);
  }

  public void lowerClimber() {
    leftArmSolenoid.set(false);
    rightArmSolenoid.set(false);
  }
}

