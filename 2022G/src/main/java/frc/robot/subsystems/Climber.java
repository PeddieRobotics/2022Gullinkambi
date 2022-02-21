package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.RobotMapGullinkambi;

public class Climber extends SubsystemBase {
  private static Climber climber;
  private CANSparkMax armPrimary, armSecondary;
  private DigitalInput armSensor;

  private double climberSetpoint;

  private SparkMaxPIDController climberPIDController;

  private double kP = Constants.CLIMBER_P;
  private double kI = Constants.CLIMBER_I;
  private double kD = Constants.CLIMBER_D;
  private double kIz = Constants.CLIMBER_IZONE;
  private double kFF = Constants.CLIMBER_FF;

  public Climber() {
    armPrimary = new CANSparkMax(RobotMapGullinkambi.MOTOR_CLIMBER_PRIMARY, MotorType.kBrushless);
    armSecondary = new CANSparkMax(RobotMapGullinkambi.MOTOR_CLIMBER_SECONDARY, MotorType.kBrushless);
    armSecondary.follow(armPrimary);

    armPrimary.setIdleMode(IdleMode.kBrake);
    armSecondary.setIdleMode(IdleMode.kBrake);
    armPrimary.setSmartCurrentLimit(Constants.CLIMBER_MAX_CURRENT);
    armSecondary.setSmartCurrentLimit(Constants.CLIMBER_MAX_CURRENT);

    armSensor = new DigitalInput(2);

    climberPIDController = armPrimary.getPIDController();
    climberPIDController.setOutputRange(-1, 1);
    enablePIDController();
  }

  public void enablePIDController(){
    climberPIDController.setP(Constants.CLIMBER_P);
    climberPIDController.setI(Constants.CLIMBER_I);
    climberPIDController.setD(Constants.CLIMBER_D);
    climberPIDController.setIZone(Constants.CLIMBER_IZONE);
    climberPIDController.setFF(Constants.CLIMBER_FF);
  }

  public void disablePIDController(){
    climberPIDController.setP(0);
    climberPIDController.setI(0);
    climberPIDController.setD(0);
    climberPIDController.setIZone(0);
    climberPIDController.setFF(0);
  }

  public static Climber getInstance() {
    if (climber == null) {
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
      armPrimary.set(speed);
    }
    else{
      if(!armSensorState()){
        armPrimary.set(speed);
      }
      else armPrimary.set(0);
    }
  }

  public double getEncoderPosition(){
    return armPrimary.getEncoder().getPosition();
  }

  public void setEncoderPosition(double position){
    armPrimary.getEncoder().setPosition(position);
  }

  public void setBrake() {
    armPrimary.setIdleMode(IdleMode.kBrake);
    armSecondary.setIdleMode(IdleMode.kBrake);
  }

  public void setCoast() {
    armPrimary.setIdleMode(IdleMode.kCoast);
    armSecondary.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    if(armSensorState()){
      setEncoderPosition(0);
    }
  }

  public void putSmartDashboardOverrides(){  
    SmartDashboard.putNumber("OR: Climber power", 0);
    SmartDashboard.putNumber("OR: Climber coast", 0);

    SmartDashboard.putNumber("OR: P climber", Constants.CLIMBER_P);
    SmartDashboard.putNumber("OR: I climber", Constants.CLIMBER_I);
    SmartDashboard.putNumber("OR: D climber", Constants.CLIMBER_D);
    SmartDashboard.putNumber("OR: I zone climber", Constants.CLIMBER_IZONE);
    SmartDashboard.putNumber("OR: FF climber", Constants.CLIMBER_FF);

    if(climber.getEncoderPosition() <= 0){
      SmartDashboard.putNumber("OR: Climber setpoint", climber.getEncoderPosition());
    }
    else{
      SmartDashboard.putNumber("OR: Climber setpoint", 0.0);
    }
  }

  public void updateClimberInfoOnDashboard(){
    SmartDashboard.putBoolean("Climber sensor state", climber.armSensorState());
    SmartDashboard.putNumber("Climber encoder", climber.getEncoderPosition());
    
  }

  public void updateClimberFromDashboard() {
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
    //climber.run(SmartDashboard.getNumber("OR: Climber power",0));

    moveToPosition(SmartDashboard.getNumber("OR: Climber setpoint", 0.0));

  }
}
