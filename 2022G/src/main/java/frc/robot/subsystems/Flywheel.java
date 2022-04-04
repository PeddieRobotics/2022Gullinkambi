
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.UpdateLogs;
import frc.robot.utils.RobotMapGullinkambi;

public class Flywheel extends SubsystemBase {
  private CANSparkMax flywheelPrimary, flywheelSecondary;

  private SparkMaxPIDController flywheelPIDController;
  private RelativeEncoder flywheelEncoder;

  private static Flywheel flywheel;

  private double kP, kI, kD, kIz, kFF;

  private Solenoid hoodSolenoid, shooterLockSolenoid;

  private SimpleMotorFeedforward flywheelFF;

  private double flywheelSetpoint = 0;

  private static UpdateLogs updateLogs = UpdateLogs.getInstance();


  public Flywheel() {
    // Set up flywheel motors
    flywheelPrimary = new CANSparkMax(RobotMapGullinkambi.MOTOR_FLYWHEEL_PRIMARY, MotorType.kBrushless);
    flywheelSecondary = new CANSparkMax(RobotMapGullinkambi.MOTOR_FLYWHEEL_SECONDARY, MotorType.kBrushless);

    flywheelPrimary.setInverted(true);
    flywheelSecondary.follow(flywheelPrimary, true);

    flywheelPrimary.setSmartCurrentLimit(Constants.FLYWHEEL_MAX_CURRENT);
    flywheelSecondary.setSmartCurrentLimit(Constants.FLYWHEEL_MAX_CURRENT);

    flywheelPIDController = flywheelPrimary.getPIDController();
    flywheelEncoder = flywheelPrimary.getEncoder();

    // Configure PID controller for the flywheel
    flywheelPIDController.setP(Constants.FLYWHEEL_P);
    flywheelPIDController.setI(Constants.FLYWHEEL_I);
    flywheelPIDController.setD(Constants.FLYWHEEL_D);
    flywheelPIDController.setIZone(Constants.FLYWHEEL_IZONE);
    flywheelPIDController.setFF(Constants.FLYWHEEL_FF);
    flywheelPIDController.setOutputRange(0, 1);

    flywheelFF = new SimpleMotorFeedforward(Constants.ksFlywheel, Constants.kvFlywheel, Constants.kaFlywheel);

    // Set up pneumatics
    hoodSolenoid = new Solenoid(RobotMapGullinkambi.PNEUMATICS_HUB, PneumaticsModuleType.REVPH, RobotMapGullinkambi.SOLENOID_HOOD);
    shooterLockSolenoid = new Solenoid(RobotMapGullinkambi.PNEUMATICS_HUB, PneumaticsModuleType.REVPH, RobotMapGullinkambi.SOLENOID_SHOOTER_LOCK);
  }

  @Override
  public void periodic() {
    if(Constants.USE_LOGGING){
      updateLogs.updateFlywheelLogData();
    }
  }

  public static Flywheel getInstance() {
    if (flywheel == null) {
      flywheel = new Flywheel();
    }
    return flywheel;
  }

  public void runFlywheelSetpoint(double rpm) {
    flywheelSetpoint = rpm;
    // bounds may need to be changed based on desired limits
    if (flywheelSetpoint > Constants.FLYWHEEL_MAX_RPM) {
      flywheelSetpoint = 0;
    }
    flywheelPIDController.setReference(flywheelSetpoint, ControlType.kVelocity, 0, flywheelFF.calculate(flywheelSetpoint/60.0));
  }

  public void runFlywheelPower(double power) {
    if (power > 0 && power < Constants.FLYWHEEL_MAX_POWER) {
      flywheelPrimary.set(power);
    } else {
      flywheelPrimary.set(0);
    }
  }

  public boolean isAtRPM(double threshold){
    if(getFlywheelSetpoint() > 0){
        return Math.abs(getFlywheelVelocity()-getFlywheelSetpoint()) < threshold;
    }
    return false;
  }

  public void setHood(boolean isUp) {
    hoodSolenoid.set(isUp);
  }

  public void setShooterLock(boolean isActivated) {
    shooterLockSolenoid.set(isActivated);
  }

  public boolean getHood() {
    return hoodSolenoid.get();
  }

  public boolean getShooterLock() {
    return shooterLockSolenoid.get();
  }

  public void stopFlywheel() {
    flywheelSetpoint = 0;
    flywheelPIDController.setReference(flywheelSetpoint, ControlType.kVelocity);
    flywheel.setShooterLock(false);
    flywheel.setHood(false);
  }

  public void putSmartDashboardOverrides() {
    SmartDashboard.putNumber("OR: Flywheel power", 0);
    SmartDashboard.putNumber("OR: Flywheel setpoint", 0);
    SmartDashboard.putBoolean("OR: Flywheel lock", false);
    SmartDashboard.putBoolean("OR: Hood up", false);

    SmartDashboard.putBoolean("LL Shot Override", false);
    
    SmartDashboard.putNumber("Teleop: shoot low RPM", Constants.FLYWHEEL_RPM_LOW);
    SmartDashboard.putNumber("Teleop: layup RPM", Constants.FLYWHEEL_RPM_LAYUP);
    SmartDashboard.putNumber("Teleop: shootLL RPM delta", 0);

    SmartDashboard.putBoolean("RevUp AI", true);

    // Smart dashboard controls for flywheel PID gain tuning
    SmartDashboard.putNumber("OR: P flywheel", Constants.FLYWHEEL_P);
    SmartDashboard.putNumber("OR: I flywheel", Constants.FLYWHEEL_I);
    SmartDashboard.putNumber("OR: D flywheel", Constants.FLYWHEEL_D);
    SmartDashboard.putNumber("OR: I zone flywheel", Constants.FLYWHEEL_IZONE);
    SmartDashboard.putNumber("OR: FF flywheel", Constants.FLYWHEEL_FF);
  }

  public void updateFlywheelInfoOnDashboard(){
    SmartDashboard.putNumber("FW velocity", getFlywheelVelocity());
    SmartDashboard.putNumber("FW setpoint", getFlywheelSetpoint());
    SmartDashboard.putBoolean("Lock activated", getShooterLock());
    SmartDashboard.putBoolean("Hood up", getHood());
    SmartDashboard.putBoolean("isAtRPM", isAtRPM(Constants.FLYWHEEL_THRESHOLD_SHOOTLL));
  }

  //Setpoint Getters
  public double getFlywheelSetpoint() {
    return flywheelSetpoint;
  }


  public double getFlywheelVelocity(){
    return flywheelEncoder.getVelocity();
  }

  //Current Getters
  public double getPrimaryFlywheelCurrent(){
    return flywheelPrimary.getOutputCurrent();
  }

  public double getSecondaryFlywheelCurrent(){
    return flywheelSecondary.getOutputCurrent();
  }

  //Motor Temperature Getters
  public double getPrimaryFlywheelMotorTemperature(){
    return flywheelPrimary.getMotorTemperature();
  }

  public double getSecondaryFlywheelMotorTemperature(){
    return flywheelSecondary.getMotorTemperature();
  }

  //Hood Getters
  public boolean isHoodUp(){
    return hoodSolenoid.get();
  }

  public void updateFlywheelPIDFromDashboard(){
    flywheelPIDController.setP(SmartDashboard.getNumber("OR: P flywheel", Constants.FLYWHEEL_P), 0);
    flywheelPIDController.setI(SmartDashboard.getNumber("OR: I flywheel", Constants.FLYWHEEL_I), 0);
    flywheelPIDController.setD(SmartDashboard.getNumber("OR: D flywheel", Constants.FLYWHEEL_D), 0);
    flywheelPIDController.setIZone(SmartDashboard.getNumber("OR: I zone flywheel", Constants.FLYWHEEL_IZONE), 0);
    flywheelPIDController.setFF(SmartDashboard.getNumber("OR: FF flywheel", Constants.FLYWHEEL_FF), 0);

  }

  public void updateFlywheelFromDashboard() {
    updateFlywheelPIDFromDashboard();
    
    setShooterLock(SmartDashboard.getBoolean("OR: Flywheel lock", false));
    setHood(SmartDashboard.getBoolean("OR: Hood up", false));
    if(SmartDashboard.getNumber("OR: Flywheel power", 0) > 0){
      runFlywheelPower(SmartDashboard.getNumber("OR: Flywheel power", 0));
    }
    else{
      runFlywheelSetpoint(SmartDashboard.getNumber("OR: Flywheel setpoint", 0));
    }

  }

}
