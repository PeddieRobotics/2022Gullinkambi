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
  private CANSparkMax flywheelPrimary, flywheelSecondary;

  private SparkMaxPIDController flywheelPIDController;
  private RelativeEncoder flywheelEncoder;

  private static Flywheel flywheel;

  private double kP = Constants.FLYWHEEL_P;
  private double kI = Constants.FLYWHEEL_I;
  private double kD = Constants.FLYWHEEL_D;
  private double kIz = Constants.FLYWHEEL_IZONE;
  private double kFF = Constants.FLYWHEEL_FF;

  private Solenoid hoodSolenoid, shooterLockSolenoid;

  private double flywheelSetpoint = 0;
  private double flywheelPower = 0;

  public Flywheel() {
    // Set up flywheel motors
    flywheelPrimary = new CANSparkMax(RobotMap.MOTOR_FLYWHEEL_PRIMARY, MotorType.kBrushless);
    flywheelSecondary = new CANSparkMax(RobotMap.MOTOR_FLYWHEEL_SECONDARY, MotorType.kBrushless);

    flywheelSecondary.follow(flywheelPrimary, false);

    flywheelPIDController = flywheelPrimary.getPIDController();
    flywheelEncoder = flywheelPrimary.getEncoder();

    // Configure PID controller for the flywheel
    flywheelPIDController.setP(kP);
    flywheelPIDController.setI(kI);
    flywheelPIDController.setD(kD);
    flywheelPIDController.setIZone(kIz);
    flywheelPIDController.setFF(kFF);
    flywheelPIDController.setOutputRange(0, 1);

    // Set up pneumatics
    hoodSolenoid = new Solenoid(RobotMap.PNEUMATICS_HUB, PneumaticsModuleType.REVPH, RobotMap.SOLENOID_HOOD);
    shooterLockSolenoid = new Solenoid(RobotMap.PNEUMATICS_HUB, PneumaticsModuleType.REVPH, RobotMap.SOLENOID_SHOOTER_LOCK);
    setShooterLock(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static Flywheel getInstance() {

    if (flywheel == null) {
      flywheel = new Flywheel();
    }
    return flywheel;
  }

  public void runFlywheelSetPoint(double rpm) {
    flywheelSetpoint = rpm;
    // bounds may need to be changed based on desired limits
    if (flywheelSetpoint > Constants.FLYWHEEL_MAX_RPM) {
      flywheelSetpoint = 0;
    }
    flywheelPIDController.setReference(flywheelSetpoint, ControlType.kVelocity);
  }

  public void runFlyWheelPower(double power) {
    if (power > 0 && power < Constants.FLYWHEEL_MAX_POWER) {
      flywheelPrimary.set(power);
    } else {
      flywheelPrimary.set(0);
    }

  }

  public boolean isAtRPM(double threshold) {
    if (getFlywheelSetpoint() > 0) {
      return Math.abs(getFlywheelVelocity() - getFlywheelSetpoint()) < threshold;
    }
    return false;
  }

  public void setHood(boolean isUp) {
    hoodSolenoid.set(isUp);
  }

  public void setShooterLock(boolean isActivated) {
    shooterLockSolenoid.set(isActivated);
  }

  public void getHood() {
    hoodSolenoid.get();
  }

  public void getShooterLock() {
    shooterLockSolenoid.get();
  }

  public void stopFlywheel() {
    setHood(false);
    setShooterLock(false);
    flywheelPIDController.setReference(0, ControlType.kVelocity);
  }

  public double getFlywheelSetpoint() {
    return flywheelSetpoint;
  }

  public double getFlywheelVelocity() {
    return flywheelPrimary.getEncoder().getVelocity();
  }

  public boolean isHoodUp() {
    return hoodSolenoid.get();
  }

  public void putSmartDashboardOverrides() {
    SmartDashboard.putNumber("OR: Flywheel power", 0);
    SmartDashboard.putNumber("OR: Flywheel setpoint", 0);
    SmartDashboard.putBoolean("OR: Hood up", false);
    SmartDashboard.putBoolean("OR: Flywheel Lock", false);

    // Smart dashboard controls for flywheel PID gain tuning
    SmartDashboard.putNumber("OR: P gain", kP);
    SmartDashboard.putNumber("OR: I gain", kI);
    SmartDashboard.putNumber("OR: D gain", kD);
    SmartDashboard.putNumber("OR: I zone", kIz);
    SmartDashboard.putNumber("OR: Feed forward", kFF);
  }

  public void updateFlywheelFromDashboard() {
    flywheelPIDController.setP(SmartDashboard.getNumber("OR: P gain", kP));
    flywheelPIDController.setI(SmartDashboard.getNumber("OR: I gain", kI));
    flywheelPIDController.setD(SmartDashboard.getNumber("OR: D gain", kD));
    flywheelPIDController.setIZone(SmartDashboard.getNumber("OR: I zone", kIz));
    flywheelPIDController.setFF(SmartDashboard.getNumber("OR: Feed forward", kFF));

    setShooterLock(SmartDashboard.getBoolean("OR: Flywheel Lock", false));
    setHood(SmartDashboard.getBoolean("OR: Hood up", false));
    runFlywheelSetPoint(SmartDashboard.getNumber("OR: Flywheel setpoint", 0));
    runFlyWheelPower(SmartDashboard.getNumber("OR: Flywheel power", 0));
  }

}
