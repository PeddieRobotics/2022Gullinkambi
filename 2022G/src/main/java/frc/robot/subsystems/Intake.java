// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotMap;

public class Intake extends SubsystemBase {

  private static Intake intake;

  private Solenoid intakeSolenoid;
  private VictorSP intakeMotor;

  public Intake() {
    intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.SOLENOID_INTAKE);
    // intakeMotor = new VictorSP(RobotMap.MOTOR_INTAKE);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static Intake getInstance() {
    if (intake == null) {
      intake = new Intake();
    }

    return intake;
  }

  public void setIntakeSpeed(double intakeSpeed) {
    intakeMotor.set(intakeSpeed);
  }

  public double getIntakeSpeed() {
    return (intakeMotor.get());
  }

  public void setIntakeSolenoid(boolean solenoidState) {
    intakeSolenoid.set(solenoidState);
  }

  public boolean getIntakeSolenoid() {
    return intakeSolenoid.get();
  }

  public void runIntake(double speed) {
    intakeMotor.set(speed);
    intakeSolenoid.set(true);
  }

  public void stopIntake() {
    intakeMotor.set(0);
    intakeSolenoid.set(false);
  }

  public void reverseIntake(double speed) {
    runIntake(-speed);
    intakeSolenoid.set(true);
  }

  public boolean isIntaking() {
    return (intakeMotor.get() > 0.0);
  }

  public void putSmartDashboardOverrides() {
    // SmartDashboard.putNumber("OR: Intake speed", getIntakeSpeed());
    SmartDashboard.putBoolean("OR: Intake solenoid", getIntakeSolenoid());
  }

  public void updateIntakeFromDashboard() {
    // setIntakeSpeed(SmartDashboard.getNumber("OR: Intake speed", getIntakeSpeed()));
    setIntakeSolenoid(SmartDashboard.getBoolean("OR: Intake solenoid", getIntakeSolenoid()));
  }
}