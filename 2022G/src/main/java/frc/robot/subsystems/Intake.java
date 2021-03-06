// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.OIConfig;
import frc.robot.utils.RobotMapGullinkambi;

public class Intake extends SubsystemBase {

  private static Intake intake;

  private Solenoid intakeSolenoid;
  private CANSparkMax intakeMotor;

  public Intake() {
    intakeSolenoid = new Solenoid(RobotMapGullinkambi.PNEUMATICS_HUB, PneumaticsModuleType.REVPH, RobotMapGullinkambi.SOLENOID_INTAKE);
    intakeMotor = new CANSparkMax(RobotMapGullinkambi.MOTOR_INTAKE, MotorType.kBrushed);

    intakeMotor.setSmartCurrentLimit(Constants.INTAKE_MAX_CURRENT);
  }

  @Override
  public void periodic() {

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

  public void stopIntake() {
    intakeMotor.set(0);
    intakeSolenoid.set(false);
  }

  public void reverseIntake(double speed) {
    setIntakeSpeed(-speed);
  }

  public boolean isIntaking() {
    return (intakeMotor.get() > 0.0);
  }

  public void putSmartDashboardOverrides() {
    SmartDashboard.putNumber("OR: Intake speed", 0.0);
    SmartDashboard.putBoolean("OR: Intake solenoid", false);
    SmartDashboard.putNumber("Teleop: Intake speed", Constants.INTAKE_SPEED);
  }

  public void updateIntakeInfoOnDashboard(){
    SmartDashboard.putBoolean("Intake solenoid", getIntakeSolenoid());
    SmartDashboard.putNumber("Intake speed", getIntakeSpeed());
    if(Constants.OI_CONFIG != OIConfig.COMPETITION){
      SmartDashboard.putNumber("Intake current", getIntakeCurrent());
    }

  }

  public void updateIntakeFromDashboard() {
    if (getIntakeSolenoid()){
    setIntakeSpeed(SmartDashboard.getNumber("OR: Intake speed", 0.0));
    } else {
      stopIntake();
    }
    
    setIntakeSolenoid(SmartDashboard.getBoolean("OR: Intake solenoid", false));
  }

  public void putValuesSmartDashboard() {
    SmartDashboard.putNumber("Intake speed", getIntakeSpeed());
    SmartDashboard.putBoolean("Intake solenoid", getIntakeSolenoid());
  }

  //Getters
  public boolean getSolenoidState() {
    return intakeSolenoid.get();
  }

  public double getIntakeCurrent(){
    return intakeMotor.getOutputCurrent();
  }

  public double getIntakeMotorTemperature(){
    return intakeMotor.getMotorTemperature();
  }
}