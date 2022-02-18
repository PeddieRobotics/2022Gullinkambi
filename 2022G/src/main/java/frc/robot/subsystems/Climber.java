package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.*;

public class Climber extends SubsystemBase {
  private static Climber climber;
  

  private static UpdateLogs updateLogs = UpdateLogs.getInstance();

  private CANSparkMax armMotor1, armMotor2;
  private DigitalInput armSensor;

  private double climberSetpoint;

  private SparkMaxPIDController climberPIDController;

  private double kP = Constants.CLIMBER_P;
  private double kI = Constants.CLIMBER_I;
  private double kD = Constants.CLIMBER_D;
  private double kIz = Constants.CLIMBER_IZONE;
  private double kFF = Constants.CLIMBER_FF;

  public Climber() {
    armMotor1 = new CANSparkMax(RobotMapGullinkambi.MOTOR_CLIMBER_PRIMARY, MotorType.kBrushless);
    armMotor2 = new CANSparkMax(RobotMapGullinkambi.MOTOR_CLIMBER_SECONDARY, MotorType.kBrushless);
    armMotor2.follow(armMotor1);

    armMotor1.setIdleMode(IdleMode.kBrake);
    armMotor2.setIdleMode(IdleMode.kBrake);
    armMotor1.setSmartCurrentLimit(Constants.CLIMBER_MAX_CURRENT);
    armMotor2.setSmartCurrentLimit(Constants.CLIMBER_MAX_CURRENT);

    armSensor = new DigitalInput(2);

    climberPIDController = armMotor1.getPIDController();
    climberPIDController.setP(Constants.CLIMBER_P);
    climberPIDController.setI(Constants.CLIMBER_I);
    climberPIDController.setD(Constants.CLIMBER_D);
    climberPIDController.setIZone(Constants.CLIMBER_IZONE);
    climberPIDController.setFF(Constants.CLIMBER_FF);
    climberPIDController.setOutputRange(0, 1);
  }
  
  public static Climber getInstance(){
    if(climber == null){
      climber = new Climber();
      climber.register();
    }
    return climber;
  }

  public boolean armSensorState(){
    return !armSensor.get();  
  }

  public void moveToPosition(double encoderPosition){
    climberSetpoint = encoderPosition;
    climberPIDController.setReference(climberSetpoint, ControlType.kPosition);
  }

  public void run(double speed) {
    if(speed < 0){
      armMotor1.set(speed);
    }
    else{
      if(!armSensorState()){
        armMotor1.set(speed);
      }
      else armMotor1.set(0);
    }
  }

  public double getEncoderPosition(){
    return armMotor1.getEncoder().getPosition();
  }

  public void setEncoderPosition(double position){
    armMotor1.getEncoder().setPosition(position);
  }

  public void setBrake() {
    armMotor1.setIdleMode(IdleMode.kBrake);
    armMotor2.setIdleMode(IdleMode.kBrake);
  }

  public void setCoast() {
    armMotor1.setIdleMode(IdleMode.kCoast);
    armMotor2.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    updateLogs.updateClimberLogData();
  }

  public double getarmMotor1Velocity(){
    return armMotor1.get();
  }
  
  public double getarmMotor2Velocity(){
    return armMotor2.get();
  }

  public double getarmMotor1Current(){
    return armMotor1.getOutputCurrent();
  }
  
  public double getarmMotor2Current(){
    return armMotor2.getOutputCurrent();
  }

  public double getarmMotor1Temperature(){
    return armMotor1.getMotorTemperature();
  }
  
  public double getarmMotor2Temperature(){
    return armMotor2.getMotorTemperature();
  }

  public double getarmMotor1EncoderVelocity(){
    return armMotor1.getEncoder().getVelocity();
  }

  public double getarmMotor1EncoderPosition(){
    return armMotor1.getEncoder().getPosition();
  }

  public boolean sensesArm(){
    return armSensor.get();
  }

  public void putSmartDashboardOverrides(){
    //SmartDashboard.putNumber("Climber encoder", climber.getEncoderPosition());
  
    SmartDashboard.putNumber("OR: Climber power", 0);
    SmartDashboard.putNumber("OR: Climber coast", 0);

    SmartDashboard.putNumber("OR: P climber", 0);
    SmartDashboard.putNumber("OR: I climber", 0);
    SmartDashboard.putNumber("OR: D climber", 0);
    SmartDashboard.putNumber("OR: I zone climber", 0);
    SmartDashboard.putNumber("OR: FF climber", 0);

    SmartDashboard.putNumber("OR: Climber setpoint", climber.getEncoderPosition());
  }

  public void updateClimberInfoOnDashboard(){
    SmartDashboard.putBoolean("Climber sensor state", climber.armSensorState());
    SmartDashboard.putNumber("Climber encoder", climber.getEncoderPosition());
    
  }

  public void updateClimberFromDashboard() {
    climber.run(SmartDashboard.getNumber("OR: Climber power",0));
    if(SmartDashboard.getBoolean("OR: Climber coast", false)){
      climber.setCoast();
    }
    else{
      climber.setBrake();
    }
    
    climberPIDController.setP(SmartDashboard.getNumber("OR: P climber", kP));
    climberPIDController.setI(SmartDashboard.getNumber("OR: I climber", kI));
    climberPIDController.setD(SmartDashboard.getNumber("OR: D climber", kD));
    climberPIDController.setIZone(SmartDashboard.getNumber("OR: I zone climber", kIz));
    climberPIDController.setFF(SmartDashboard.getNumber("OR: FF climber", kFF));
    
    if(armSensorState()){
      setEncoderPosition(0);
    }
    moveToPosition(SmartDashboard.getNumber("OR: Climber setpoint", 0.0));

  }
}
