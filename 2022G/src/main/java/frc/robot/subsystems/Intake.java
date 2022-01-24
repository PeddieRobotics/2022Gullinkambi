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

  public Intake(){
    intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.SOLENOID_INTAKE);
    intakeMotor = new VictorSP(RobotMap.MOTOR_INTAKE);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static Intake getInstance(){
    if (intake == null){
      intake = new Intake();
    }

    return intake;
  }

  public void runIntake(double speed, boolean solenoidState){
    intakeMotor.set(speed);
    intakeSolenoid.set(solenoidState);
  }

  public void stopIntake(){
    runIntake(0, false);
  }

  public void reverseIntake(double speed){
    runIntake(-speed, true);
  }

  public boolean isIntaking() {
     return (intakeMotor.get() > 0.0);
  }
  
  public boolean getSolenoidState() {
    return intakeSolenoid.get();
  }

  public double getSpeed() {
    return(intakeMotor.get());
  }

  public void putSmartDashboardOverrides() {
    SmartDashboard.putNumber("OR: Intake speed", getSpeed());
    SmartDashboard.putBoolean("OR: Intake solenoid", getSolenoidState());
  }
}