package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.commands.FollowTarget;
import frc.robot.utils.RobotMap;
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

public class Turret extends SubsystemBase {
  private static Turret turret;

  private CANSparkMax spark;
  private SparkMaxPIDController pidController;
  private RelativeEncoder encoder;

  public enum TurretMode{
    INTAKING, TARGETING, OVERRIDING
  };
  private TurretMode mode = TurretMode.TARGETING;
  public TurretMode previousMode;
  private double kP = 0.00005;
  private double kI = 0.000001;
  private double kD = 0.0;
  private double kIz = 0.0;
  private double kFF = 0.000156;
  private double kMaxOutput = 1.0;
  private double kMinOutput = -1.0;

  private double maxVel = 4000; //rpm
  private double minVel = 0.0; 
  private double maxAcc = 3000; 
  private double allowedErr = 0.0;
  private int smartMotionSlot = 0;
  private int ballsInTower = 0;

  private double setPoint;  
  private double distancePerPulse = 0.5787;

  private double limelightErrorScaleFactor = 0.1;
  private double limelightErrorMaxTol = 2.0;

  private double angleMax=90;
  private double angleMin=0;

  public Turret() {
    

    spark = new CANSparkMax(RobotMap.TURRET_MOTOR, MotorType.kBrushless); 
    spark.setIdleMode(IdleMode.kBrake);

    pidController = spark.getPIDController();
    encoder = spark.getEncoder();
    encoder.setPosition(0.0);

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);    
  }

  public static Turret getInstance(){
    if(turret == null){
      turret = new Turret();
      turret.register();
    }
    
    return turret;
  }

  /**
   * @param setpoint the angle to set the arm to, with the vertical as 0
   */
  public void setTurret(double setpoint) {
    // Only try to move the turret if the brake is released and the angle is in an allowable range
    if(setpoint >=angleMin && setpoint <=angleMax ){
      setPoint = setpoint * distancePerPulse;
      pidController.setReference(setPoint, ControlType.kSmartMotion);
    }
  }

  /**
   * @param idleMode the mode for the motor to go into when idle (brake/coast)
   */
  public void setIdleBrakeMode(boolean idleMode) {
    if(idleMode) {
      spark.setIdleMode(IdleMode.kBrake);
    } else {
      spark.setIdleMode(IdleMode.kCoast);
    }
  }

  /**
   * @return whether the arm is at the target angle or not
   */
  public boolean atTarget() {
    return (Math.abs((encoder.getPosition() - setPoint)) < 1.5) && (Math.abs(encoder.getVelocity()) < 25.0);
  }

  public double setPoint(){
    return setPoint;
  }
  
  public double getCurrentAngle(){
    return setPoint/distancePerPulse;
  }

  public double getLimelightErrorScaleFactor(){
    return limelightErrorScaleFactor;
  }

  public double getLimelightErrorMaxTol(){
    return limelightErrorMaxTol;
  }

  public void setOverride(){
    previousMode = mode;
    mode = TurretMode.OVERRIDING;
  }
  public void setPreviousMode(){
    mode = previousMode;
    previousMode = TurretMode.OVERRIDING;
    if(mode == TurretMode.TARGETING){
      turret.runFollowTarget();
    }
    else if (mode == TurretMode.INTAKING){
      turret.setTurret(0);
    } 
  }
  public TurretMode getMode(){
    return mode;
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("XBOX Input", OI.getInstance().getTurretInputFromThumbstick());
    SmartDashboard.putString("Turret Mode", this.getMode().toString());

    SmartDashboard.putNumber("Turret angle", this.getCurrentAngle());
    SmartDashboard.putNumber("Turret vel",this.encoder.getVelocity()/70);
    maxVel = SmartDashboard.getNumber("Max turret vel (rpm)", 0);
    maxAcc = SmartDashboard.getNumber("Max turret accel (rpm/s)", 0);
    pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    limelightErrorScaleFactor = SmartDashboard.getNumber("LL Error Scale Factor", 0.1);
    limelightErrorMaxTol = SmartDashboard.getNumber("LL Max Error Tol", 2.0);
    angleMax = SmartDashboard.getNumber("Angle Max", 90);
    angleMin= SmartDashboard.getNumber("Angle Min",0 );
    ballsInTower = (int)SmartDashboard.getNumber("Balls in Tower",0);

    // If we get sufficient signal from the thumbstick, start using its signal to control the turret
    switch(mode){
      case INTAKING:
        // If the intake stops, start following target with LL via a command
        if(!SmartDashboard.getBoolean("Intake", false) || ballsInTower==2){
          previousMode = TurretMode.INTAKING;
          mode = TurretMode.TARGETING;
          SmartDashboard.putBoolean("Intake", false);
          //turret.runFollowTarget();//implement differently
        }
        break; 
      case TARGETING:
        // If the intake starts, stop following target and reset to neutral position
        if(SmartDashboard.getBoolean("Intake", false)|| ballsInTower==0){
          previousMode = TurretMode.TARGETING;
          mode = TurretMode.INTAKING;
          SmartDashboard.putBoolean("Intake", true);
          this.setTurret(0.0);
        }
        break;
      case OVERRIDING:
        // If we are overriding the normal turret behavior from the controls, make updates to the turret angle based on
        // some scale factor of the thumbstick input. Here 5.0 has been chosen arbitrary for test purposes...
        
        //this.setTurret(this.getCurrentAngle() + OI.getInstance().getTurretInputFromThumbstick() * 5.0);    
    };
  }
}