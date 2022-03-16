package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.*;

public class Climber extends SubsystemBase {
  private static Climber climber;
  
  private static UpdateLogs updateLogs = UpdateLogs.getInstance();

  private CANSparkMax rightArmMotorPrimary, rightArmMotorSecondary, leftArmMotorPrimary, leftArmMotorSecondary;
  private DigitalInput rightArmSensor, leftArmSensor;
  private Solenoid rightArmBrake, leftArmBrake, thirdArmExtender; 

  private double climberRightSetpoint, climberLeftSetpoint;

  private SparkMaxPIDController climberRightPIDController, climberLeftPIDController;

  private double kP = Constants.CLIMBER_P;
  private double kI = Constants.CLIMBER_I;
  private double kD = Constants.CLIMBER_D;
  private double kIz = Constants.CLIMBER_IZONE;
  private double kFF = Constants.CLIMBER_FF;

  public Climber() {
    rightArmMotorPrimary = new CANSparkMax(RobotMapGullinkambi.MOTOR_CLIMBER_RIGHT_PRIMARY, MotorType.kBrushless);
    rightArmMotorSecondary = new CANSparkMax(RobotMapGullinkambi.MOTOR_CLIMBER_RIGHT_SECONDARY, MotorType.kBrushless);
    rightArmMotorSecondary.follow(rightArmMotorPrimary);

    rightArmMotorPrimary.setIdleMode(IdleMode.kBrake);
    rightArmMotorSecondary.setIdleMode(IdleMode.kBrake);
    rightArmMotorPrimary.setSmartCurrentLimit(Constants.CLIMBER_MAX_CURRENT);
    rightArmMotorSecondary.setSmartCurrentLimit(Constants.CLIMBER_MAX_CURRENT);

    leftArmMotorPrimary = new CANSparkMax(RobotMapGullinkambi.MOTOR_CLIMBER_LEFT_PRIMARY, MotorType.kBrushless);
    leftArmMotorSecondary = new CANSparkMax(RobotMapGullinkambi.MOTOR_CLIMBER_LEFT_SECONDARY, MotorType.kBrushless);
    leftArmMotorSecondary.follow(leftArmMotorPrimary);

    leftArmMotorPrimary.setIdleMode(IdleMode.kBrake);
    leftArmMotorSecondary.setIdleMode(IdleMode.kBrake);
    leftArmMotorPrimary.setSmartCurrentLimit(Constants.CLIMBER_MAX_CURRENT);
    leftArmMotorSecondary.setSmartCurrentLimit(Constants.CLIMBER_MAX_CURRENT);

    rightArmSensor = new DigitalInput(2);
    rightArmBrake = new Solenoid(RobotMapGullinkambi.PNEUMATICS_HUB, PneumaticsModuleType.REVPH, RobotMapGullinkambi.SOLENOID_CLIMBER_RIGHT_BRAKE); 

    leftArmSensor = new DigitalInput(3);
    leftArmBrake = new Solenoid(RobotMapGullinkambi.PNEUMATICS_HUB, PneumaticsModuleType.REVPH, RobotMapGullinkambi.SOLENOID_CLIMBER_LEFT_BRAKE); 

    climberRightPIDController = rightArmMotorPrimary.getPIDController();
    climberRightPIDController.setOutputRange(-1, 1);
    enableRightPIDController();

    climberLeftPIDController = leftArmMotorPrimary.getPIDController();
    climberLeftPIDController.setOutputRange(-1, 1);
    enableLeftPIDController();

    thirdArmExtender = new Solenoid(RobotMapGullinkambi.PNEUMATICS_HUB, PneumaticsModuleType.REVPH, RobotMapGullinkambi.SOLENOID_CLIMBER_THIRD_ARM);

  }

  public void enableRightPIDController(){
    climberRightPIDController.setP(Constants.CLIMBER_P);
    climberRightPIDController.setI(Constants.CLIMBER_I);
    climberRightPIDController.setD(Constants.CLIMBER_D);
    climberRightPIDController.setIZone(Constants.CLIMBER_IZONE);
    climberRightPIDController.setFF(Constants.CLIMBER_FF);
  }

  public void disableRightPIDController(){
    climberRightPIDController.setP(0);
    climberRightPIDController.setI(0);
    climberRightPIDController.setD(0);
    climberRightPIDController.setIZone(0);
    climberRightPIDController.setFF(0);
  }

  public void enableLeftPIDController(){
    climberLeftPIDController.setP(Constants.CLIMBER_P);
    climberLeftPIDController.setI(Constants.CLIMBER_I);
    climberLeftPIDController.setD(Constants.CLIMBER_D);
    climberLeftPIDController.setIZone(Constants.CLIMBER_IZONE);
    climberLeftPIDController.setFF(Constants.CLIMBER_FF);
  }

  public void disableLeftPIDController(){
    climberLeftPIDController.setP(0);
    climberLeftPIDController.setI(0);
    climberLeftPIDController.setD(0);
    climberLeftPIDController.setIZone(0);
    climberLeftPIDController.setFF(0);
  }
  
  public static Climber getInstance(){
    if(climber == null){
      climber = new Climber();
      climber.register();
    }
    return climber;
  }

  public boolean rightArmSensorState(){
    return !rightArmSensor.get();  
  }

  public boolean leftArmSensorState(){
    return !leftArmSensor.get();  
  }

  public void moveRightArmToPosition(double encoderPosition){
    climberRightSetpoint = encoderPosition;
    climberRightPIDController.setReference(climberRightSetpoint, ControlType.kPosition);
  }

  public void moveLeftArmToPosition(double encoderPosition){
    climberLeftSetpoint = encoderPosition;
    climberLeftPIDController.setReference(climberLeftSetpoint, ControlType.kPosition);
  }

  public void moveBothArmsToPosition(double encoderPosition){
    moveRightArmToPosition(encoderPosition);
    moveLeftArmToPosition(encoderPosition);
  }

  public void setRightArmMotorSpeed(double speed) {
    if(speed < 0){
      rightArmMotorPrimary.set(speed);
    }
    else{
      if(!rightArmSensorState()){
        rightArmMotorPrimary.set(speed);
      }
      else rightArmMotorPrimary.set(0);
    }
  }

  public void setLeftArmMotorSpeed(double speed) {
    if(speed < 0){
      leftArmMotorPrimary.set(speed);
    }
    else{
      if(!leftArmSensorState()){
        leftArmMotorPrimary.set(speed);
      }
      else leftArmMotorPrimary.set(0);
    }
  }

  public void setBothArmsMotorSpeed(double speed){
    setRightArmMotorSpeed(speed);
    setLeftArmMotorSpeed(speed);
  }

  public double getRightArmEncoderPosition(){
    return rightArmMotorPrimary.getEncoder().getPosition();
  }

  public void setRightArmEncoderPosition(double position){
    rightArmMotorPrimary.getEncoder().setPosition(position);
  }

  public double getLeftArmEncoderPosition(){
    return leftArmMotorPrimary.getEncoder().getPosition();
  }

  public void setLeftArmEncoderPosition(double position){
    leftArmMotorPrimary.getEncoder().setPosition(position);
  }

  public void setBothArmsEncoderPosition(double position){
    setRightArmEncoderPosition(position);
    setLeftArmEncoderPosition(position);
  }

  public void setBrakeMode(){
    setRightArmBrakeMode();
    setLeftArmBrakeMode();
  }

  public void setCoastMode(){
    setRightArmCoastMode();
    setLeftArmCoastMode();
  }

  public void setRightArmBrakeMode() {
    rightArmMotorPrimary.setIdleMode(IdleMode.kBrake);
    rightArmMotorSecondary.setIdleMode(IdleMode.kBrake);
  }

  public void setRightArmCoastMode() {
    rightArmMotorPrimary.setIdleMode(IdleMode.kCoast);
    rightArmMotorSecondary.setIdleMode(IdleMode.kCoast);
  }

  public void setLeftArmBrakeMode() {
    leftArmMotorPrimary.setIdleMode(IdleMode.kBrake);
    leftArmMotorSecondary.setIdleMode(IdleMode.kBrake);
  }

  public void setLeftArmCoastMode() {
    leftArmMotorPrimary.setIdleMode(IdleMode.kCoast);
    leftArmMotorSecondary.setIdleMode(IdleMode.kCoast);
  }

  public void setRightArmSolenoidBrake(boolean solenoidState) {
    rightArmBrake.set(solenoidState);
  }

  public void setLeftArmSolenoidBrake(boolean solenoidState) {
    leftArmBrake.set(solenoidState);
  }

  public boolean getRightArmSolenoidBrake() {
    return rightArmBrake.get();
  }

  public boolean getLeftArmSolenoidBrake() {
    return leftArmBrake.get();
  }

  public void setThirdArmExtender(boolean state){
    thirdArmExtender.set(state);
  }

  public boolean getThirdArmExtender(){
    return thirdArmExtender.get();
  }

  @Override
  public void periodic() {
    if(Constants.USE_LOGGING){
      updateLogs.updateClimberLogData();
    }
  }

  public double getRightArmPrimaryMotorVelocity(){
    return rightArmMotorPrimary.get();
  }
  
  public double getRightArmSecondaryMotorVelocity(){
    return rightArmMotorSecondary.get();
  }

  public double getRightArmPrimaryMotorCurrent(){
    return rightArmMotorPrimary.getOutputCurrent();
  }
  
  public double getRightArmSecondaryMotorCurrent(){
    return rightArmMotorSecondary.getOutputCurrent();
  }

  public double getRightArmPrimaryMotorTemperature(){
    return rightArmMotorPrimary.getMotorTemperature();
  }
  
  public double getRightArmSecondaryMotorTemperature(){
    return rightArmMotorSecondary.getMotorTemperature();
  }

  public double getRightArmPrimaryMotorEncoderVelocity(){
    return rightArmMotorPrimary.getEncoder().getVelocity();
  }

  public double getRightArmPrimaryMotorEncoderPosition(){
    return rightArmMotorPrimary.getEncoder().getPosition();
  }

  public void putSmartDashboardOverrides(){  

    SmartDashboard.putBoolean("OR: Climber right brake", false);
    SmartDashboard.putBoolean("OR: Climber left brake", false);

    SmartDashboard.putBoolean("OR: Climber third arm", false);

    SmartDashboard.putNumber("OR: Climber (right) power", 0);
    SmartDashboard.putNumber("OR: Climber (left) power", 0);

    SmartDashboard.putNumber("OR: P climber", Constants.CLIMBER_P);
    SmartDashboard.putNumber("OR: I climber", Constants.CLIMBER_I);
    SmartDashboard.putNumber("OR: D climber", Constants.CLIMBER_D);
    SmartDashboard.putNumber("OR: I zone climber", Constants.CLIMBER_IZONE);
    SmartDashboard.putNumber("OR: FF climber", Constants.CLIMBER_FF);

    if(climber.getRightArmEncoderPosition() <= 0){
      SmartDashboard.putNumber("OR: Climber (right) setpoint", climber.getRightArmEncoderPosition());
    }
    else{
      SmartDashboard.putNumber("OR: Climber (right) setpoint", 0.0);
    }

    if(climber.getLeftArmEncoderPosition() <= 0){
      SmartDashboard.putNumber("OR: Climber (left) setpoint", climber.getLeftArmEncoderPosition());
    }
    else{
      SmartDashboard.putNumber("OR: Climber (left) setpoint", 0.0);
    }

  }

  public void updateClimberInfoOnDashboard(){
    SmartDashboard.putBoolean("Climber (left) sensor", climber.leftArmSensorState());
    SmartDashboard.putBoolean("Climber (right) sensor", climber.rightArmSensorState());
    SmartDashboard.putNumber("Climber (left) encoder", climber.getLeftArmEncoderPosition());
    SmartDashboard.putNumber("Climber (right) encoder", climber.getRightArmEncoderPosition());
    SmartDashboard.putBoolean("Climber (left) brake", climber.getLeftArmSolenoidBrake());
    SmartDashboard.putBoolean("Climber (right) brake", climber.getRightArmSolenoidBrake());
    SmartDashboard.putBoolean("Climber third arm", climber.getThirdArmExtender());
  }

  public void updateLeftClimberPIDFromDashboard(){
    climberLeftPIDController.setP(SmartDashboard.getNumber("OR: P climber", kP));
    climberLeftPIDController.setI(SmartDashboard.getNumber("OR: I climber", kI));
    climberLeftPIDController.setD(SmartDashboard.getNumber("OR: D climber", kD));
    climberLeftPIDController.setIZone(SmartDashboard.getNumber("OR: I zone climber", kIz));
    climberLeftPIDController.setFF(SmartDashboard.getNumber("OR: FF climber", kFF));
    
  }

  public void updateRightClimberPIDFromDashboard(){
    climberRightPIDController.setP(SmartDashboard.getNumber("OR: P climber", kP));
    climberRightPIDController.setI(SmartDashboard.getNumber("OR: I climber", kI));
    climberRightPIDController.setD(SmartDashboard.getNumber("OR: D climber", kD));
    climberRightPIDController.setIZone(SmartDashboard.getNumber("OR: I zone climber", kIz));
    climberRightPIDController.setFF(SmartDashboard.getNumber("OR: FF climber", kFF));
    
  }

  public void updateClimberFromDashboard() {
    updateLeftClimberPIDFromDashboard();
    updateRightClimberPIDFromDashboard();
    
    climber.setLeftArmSolenoidBrake(SmartDashboard.getBoolean("OR: Climber (left) brake", false));
    climber.setRightArmSolenoidBrake(SmartDashboard.getBoolean("OR: Climber (right) brake", false));
    climber.setThirdArmExtender(SmartDashboard.getBoolean("OR: Climber third arm", false));

    if(rightArmSensorState()){
      setRightArmEncoderPosition(0);
    }

    if(leftArmSensorState()){
      setLeftArmEncoderPosition(0);
    }

    if(SmartDashboard.getNumber("OR: Climber (right) setpoint", 0.0) > 0){
      moveRightArmToPosition(SmartDashboard.getNumber("OR: Climber (right) setpoint", 0.0));
    }
    else{
      climber.setRightArmMotorSpeed(SmartDashboard.getNumber("OR: Climber (right) power",0));
    }

    if(SmartDashboard.getNumber("OR: Climber (left) setpoint", 0.0) > 0){
      moveLeftArmToPosition(SmartDashboard.getNumber("OR: Climber (left) setpoint", 0.0));
    }
    else{
      climber.setLeftArmMotorSpeed(SmartDashboard.getNumber("OR: Climber (left) power",0));
    }
    
  }
}
