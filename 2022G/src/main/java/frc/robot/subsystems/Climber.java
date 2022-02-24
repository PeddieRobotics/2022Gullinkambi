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

  private CANSparkMax armMotor1, armMotor2;
  private DigitalInput armSensor;
  private Solenoid armBrake; 

  private double climberSetpoint;

  private SparkMaxPIDController climberPIDController;

  private double kP = Constants.CLIMBER_P;
  private double kI = Constants.CLIMBER_I;
  private double kD = Constants.CLIMBER_D;
  private double kIz = Constants.CLIMBER_IZONE;
  private double kFF = Constants.CLIMBER_FF;

  private boolean reboundFromCurrentSpike;

  public Climber() {
    armMotor1 = new CANSparkMax(RobotMapGullinkambi.MOTOR_CLIMBER_PRIMARY, MotorType.kBrushless);
    armMotor2 = new CANSparkMax(RobotMapGullinkambi.MOTOR_CLIMBER_SECONDARY, MotorType.kBrushless);
    armMotor2.follow(armMotor1);

    armMotor1.setIdleMode(IdleMode.kBrake);
    armMotor2.setIdleMode(IdleMode.kBrake);
    armMotor1.setSmartCurrentLimit(Constants.CLIMBER_MAX_CURRENT);
    armMotor2.setSmartCurrentLimit(Constants.CLIMBER_MAX_CURRENT);

    armSensor = new DigitalInput(2);

    armBrake = new Solenoid(RobotMapGullinkambi.PNEUMATICS_HUB, PneumaticsModuleType.REVPH, RobotMapGullinkambi.SOLENOID_CLIMBER_BRAKE); 

    climberPIDController = armMotor1.getPIDController();
    climberPIDController.setOutputRange(-1, 1);
    enablePIDController();

    reboundFromCurrentSpike = false;
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

  public void setBrakeMode() {
    armMotor1.setIdleMode(IdleMode.kBrake);
    armMotor2.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode() {
    armMotor1.setIdleMode(IdleMode.kCoast);
    armMotor2.setIdleMode(IdleMode.kCoast);
  }

  //solenoid = true is unlocked, solenoid = false is locked

  public void setClimberSolenoidBrake(boolean solenoidState) { // solenoidState = true is locked, solenoidState = false is unlocked
    armBrake.set(!solenoidState);
  }

  public boolean getClimberSolenoidBrake() {
    return !armBrake.get();
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
    if(armSensorState()){
    if (armMotor1.getOutputCurrent() > 30 && !reboundFromCurrentSpike){
      setEncoderPosition(0);
      moveToPosition(-15);
      reboundFromCurrentSpike = true;
    }
    
    if(armSensorState() && !reboundFromCurrentSpike){
      setEncoderPosition(0);
    }
  }
    return armSensor.get();
  }

  public void putSmartDashboardOverrides(){  

    SmartDashboard.putBoolean("OR: Climber set brake ", false);

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
    SmartDashboard.putBoolean("OR: Climber brake state", climber.getClimberSolenoidBrake());
  }

  public void updateClimberFromDashboard() {
    if(SmartDashboard.getBoolean("OR: Climber coast", false)){
      climber.setCoastMode();
    }
    else{
      climber.setBrakeMode();
    }
    
    climberPIDController.setP(SmartDashboard.getNumber("OR: P climber", kP));
    climberPIDController.setI(SmartDashboard.getNumber("OR: I climber", kI));
    climberPIDController.setD(SmartDashboard.getNumber("OR: D climber", kD));
    climberPIDController.setIZone(SmartDashboard.getNumber("OR: I zone climber", kIz));
    climberPIDController.setFF(SmartDashboard.getNumber("OR: FF climber", kFF));
    
    climber.setClimberSolenoidBrake(SmartDashboard.getBoolean("OR: Climber set brake ", false));

    if(armSensorState()){
      setEncoderPosition(0);
    }
    //climber.run(SmartDashboard.getNumber("OR: Climber power",0));
    if(SmartDashboard.getNumber("OR: Climber setpoint", 0.0)>Constants.CLIMBER_TOP_ENCODER_POSITION){
      moveToPosition(SmartDashboard.getNumber("OR: Climber setpoint", 0.0));
    }
    

  }
}
