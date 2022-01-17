// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.RobotMap;

public class Flywheel extends SubsystemBase {
  CANSparkMax flywheel1,flywheel2;
   private SparkMaxPIDController flywheelPIDController;
   private RelativeEncoder flywheelEncoder;
  private static Flywheel flywheel;
  private double kP = Constants.FLYWHEEL_P;
  private double kI = Constants.FLYWHEEL_I;
  private double kD = Constants.FLYWHEEL_D;
  private double kIz = Constants.FLYWHEEL_IZONE;
  private double kFF = Constants.FLYWHEEL_FF;
  private Solenoid hoodSolenoid;
   double flywheelSetpoint = 0;
  double flywheelPower = 0;
  //seperator
  public Flywheel() {
    setUpFlywheel();
    hoodSolenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.SOLENOID_HOOD);
  }


private void setUpFlywheel() {
  flywheel1 = new CANSparkMax(7, MotorType.kBrushless);
  flywheel2 = new CANSparkMax(8, MotorType.kBrushless);
   flywheelPIDController = flywheel1.getPIDController();
  flywheelEncoder = flywheel1.getEncoder();
  //set values
  flywheelPIDController.setP(kP);
    flywheelPIDController.setI(kI);
    flywheelPIDController.setD(kD);
    flywheelPIDController.setIZone(kIz);
    flywheelPIDController.setFF(kFF);
    //smart dashboard controls for PID
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    flywheelPIDController.setOutputRange(0, 1);
  flywheel2.follow(flywheel1, true);

}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public static Flywheel rf() {
    if (flywheel == null) {
      flywheel = new Flywheel();
    }
  return flywheel;

  }
  public void runFlyWheelPower(double num) { //restriction num has to be 0<num<1
      if ((0<num) && (num<=0.2)) {
        flywheel1.set(num);
      }
      else {
        flywheel1.set(0);
      }

    }

  public boolean isAtRPM(double threshold){
    if(getFlywheelSetpoint()>0){
        return (Math.abs(getFlywheelVelocity()-getFlywheelSetpoint())<threshold);

    }
    return false;
  }

  public void setHood(boolean isUp){
    hoodSolenoid.set(isUp);
  }

  public boolean isHoodUp(){
    return hoodSolenoid.get();
  }

  public void stopFlywheel(){
    setHood(false);
    setFlywheelVelocity(0);
  }
  public void setFlywheelVelocity(double rpm) {
    flywheelSetpoint = rpm;

    flywheelPIDController.setReference(flywheelSetpoint, ControlType.kVelocity);
  }


  public void runFlywheelSetPoint(double rpm) {
    flywheelSetpoint = rpm;
    if (flywheelSetpoint < 200 || flywheelSetpoint > 4000) {
      flywheelSetpoint = 0;
    }
   flywheelPIDController.setReference(flywheelSetpoint, ControlType.kVelocity);
  }

  public double getFlywheelSetpoint() {
    return flywheelSetpoint;
  }

  public double getFlywheelVelocity() {
    return flywheel1.getEncoder().getVelocity();
  }

  public static Flywheel getInstance(){

    if (flywheel==null){
      flywheel = new Flywheel();


    }
    return flywheel;
  }
}
