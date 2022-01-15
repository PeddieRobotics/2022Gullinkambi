// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.ctre.phoenix.motorcontrol.ControlMode; - I don't know what this is so I'm leaving it here but just commenting it out 

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.RobotMap;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private static Intake intake;

  private Solenoid intakeSolenoid;
  private CANSparkMax intakeMotor;

  public Intake(){
    intakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 7);
    intakeMotor = new CANSparkMax(RobotMap.INTAKE_MOTOR, MotorType.kBrushless);

  }

  public static Intake getInstance(){
    if (intake == null){
      intake = new Intake();
    }

    return intake;
  }

  public void runIntake(double speed, boolean solenoidState){
    intakeMotor.set(speed); // I took out ControlMode.PercentOutput from right before speed, I still don't know what it does, but .set only takes one value anyways
    intakeSolenoid.set(solenoidState);
  }

  public void startIntake() {
    intakeSolenoid.set(true);
    setIntakeMotor(0.2);
  }

  private void setIntakeMotor(double setpoint) {
    intakeMotor.set(setpoint); // This also had ControlMode.PercentOutput before it
  }

  public void stopIntake(){
    runIntake(0, false);
  }

  public void reverseIntake(double speed){
    intakeMotor.set(-speed); //ControlMode.PercentOutput
  }

  
  public boolean isIntaking() {
    //SmartDashboard.putNumber("intake motor reported %", intakeMotor.getMotorOutputPercent());
     return (intakeMotor.get() > 0.0);
  }
  
}