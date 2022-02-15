package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.commands.ClimbCommands.*;
import frc.robot.utils.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
    armPrimary = new CANSparkMax(RobotMap.MOTOR_CLIMBER_PRIMARY, MotorType.kBrushless);
    armSecondary = new CANSparkMax(RobotMap.MOTOR_CLIMBER_SECONDARY, MotorType.kBrushless);
    armSecondary.follow(armPrimary);
    armPrimary.setIdleMode(IdleMode.kBrake);
    armSecondary.setIdleMode(IdleMode.kBrake);
    armSensor = new DigitalInput(2);
    climberPIDController = armPrimary.getPIDController();

    climberPIDController.setP(kP);
    climberPIDController.setI(kI);
    climberPIDController.setD(kD);
    climberPIDController.setIZone(kIz);
    climberPIDController.setFF(kFF);
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
    return armSensor.get();
  }

  public void moveToPosition(double encoderPosition){
    climberSetpoint = encoderPosition;
    // bounds may need to be changed based on desired limits
    if (climberSetpoint > Constants.CLIMBER_MAX_ENCODERS) {
      climberSetpoint = 0;
    }
    climberPIDController.setReference(climberSetpoint, ControlType.kPosition);
  }

  public void run(double speed) {
    if(speed<0){
      armPrimary.set(speed);
    }
    else{
      if(armSensor.get()){
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

  public void extend() {
    setCoast();
  }

  public void retract() {
    setBrake();
    if(!armSensor.get()){ // check snesor true and false
      armPrimary.set(-0.5); // check speed positive or negative
    }
    else armPrimary.set(0);
  }

  public void setCoastMode(boolean mode){
    if(mode){
      setCoast();
    }
    else setBrake();
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
  }

  public void updateFlywheelFromDashboard() {
    climberPIDController.setP(SmartDashboard.getNumber("OR: P gain", kP));
    climberPIDController.setI(SmartDashboard.getNumber("OR: I gain", kI));
    climberPIDController.setD(SmartDashboard.getNumber("OR: D gain", kD));
    climberPIDController.setIZone(SmartDashboard.getNumber("OR: I zone", kIz));
    climberPIDController.setFF(SmartDashboard.getNumber("OR: Feed forward", kFF));
  }
}
