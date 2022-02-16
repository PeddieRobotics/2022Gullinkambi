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
    climberPIDController.setP(Constants.CLIMBER_P);
    climberPIDController.setI(Constants.CLIMBER_I);
    climberPIDController.setD(Constants.CLIMBER_D);
    climberPIDController.setIZone(Constants.CLIMBER_IZONE);
    climberPIDController.setFF(Constants.CLIMBER_FF);
    climberPIDController.setOutputRange(0, 1);
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
    //SmartDashboard.putBoolean("Climber sensor state", climber.armSensorState());
    //SmartDashboard.putNumber("Climber encoder", climber.getEncoderPosition());
  }

  public void putSmartDashboardOverrides(){
    
    SmartDashboard.putNumber("OR: Climber power", 0);
    SmartDashboard.putNumber("OR: Climber coast", 0);

    SmartDashboard.putNumber("OR: P climber", 0);
    SmartDashboard.putNumber("OR: I climber", 0);
    SmartDashboard.putNumber("OR: D climber", 0);
    SmartDashboard.putNumber("OR: I zone climber", 0);
    SmartDashboard.putNumber("OR: FF climber", 0);

    SmartDashboard.putNumber("OR: Climber setpoint", climber.getEncoderPosition());
    SmartDashboard.putNumber("OR: Minimum Climber Encoder Limit", 0);
  }

  public void updateClimberFromDashboard() {
    SmartDashboard.putBoolean("Climber sensor state", climber.armSensorState());
    SmartDashboard.putNumber("Climber encoder", climber.getEncoderPosition());
    
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
    
    /*if(climber.getEncoderPosition()>SmartDashboard.getNumber("OR: Minimum Climber Encoder Limit", 0)){
       moveToPosition(SmartDashboard.getNumber("OR: Climber setpoint", 0.0));
    }*/

  }
}
