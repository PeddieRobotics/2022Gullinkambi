// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.oi.JoystickOI;
import frc.robot.oi.XboxOI;
import frc.robot.utils.*;
import frc.robot.utils.Constants.OIConfig;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain drivetrain;

  private final CANSparkMax leftMaster, rightMaster, leftFollower1, rightFollower1;
  private CANSparkMax rightFollower2,leftFollower2;
  private final MotorControllerGroup leftMotors, rightMotors;

  private final DifferentialDrive drive;
  private final DifferentialDriveOdometry odometry;
  private final ADIS16470_IMU gyro;

  private double headingValue;

  private boolean inverseMode;
  private boolean brakeMode;

  private boolean lockedOnTarget;

  // For autonomous, keep track of previous left/right wheel speed setpoints for estimating acceleration component of movement
  private double prevLeftWheelSpeed, prevRightWheelSpeed;
  private double prevPIDUpdateTime;

  private final RelativeEncoder leftEncoder, rightEncoder;

  private SparkMaxPIDController leftDrivePIDController, rightDrivePIDController;

  private SimpleMotorFeedforward leftDriveFF, rightDriveFF;

  private PIDController turnToAnglePIDController;

  //Logging
  private static UpdateLogs updateLogs = UpdateLogs.getInstance();
  private double speedSetpoint, turnSetpoint;

  public Drivetrain() {
    leftMaster = new CANSparkMax(RobotMapGullinkambi.MOTOR_DRIVE_LEFT_MASTER, MotorType.kBrushless);
    rightMaster = new CANSparkMax(RobotMapGullinkambi.MOTOR_DRIVE_RIGHT_MASTER, MotorType.kBrushless);
    leftFollower1 = new CANSparkMax(RobotMapGullinkambi.MOTOR_DRIVE_LEFT_FOLLOWER1, MotorType.kBrushless);
    rightFollower1 = new CANSparkMax(RobotMapGullinkambi.MOTOR_DRIVE_RIGHT_FOLLOWER1, MotorType.kBrushless);
    leftFollower2 = new CANSparkMax(RobotMapGullinkambi.MOTOR_DRIVE_LEFT_FOLLOWER2, MotorType.kBrushless);
    rightFollower2 = new CANSparkMax(RobotMapGullinkambi.MOTOR_DRIVE_RIGHT_FOLLOWER2, MotorType.kBrushless);

    leftMotors = new MotorControllerGroup(leftMaster, leftFollower1, leftFollower2);
    rightMotors = new MotorControllerGroup(rightMaster, rightFollower1, rightFollower2);

    leftFollower1.follow(leftMaster);
    leftFollower2.follow(leftMaster);
    rightFollower1.follow(rightMaster);
    rightFollower2.follow(rightMaster);

    leftMaster.setSmartCurrentLimit(Constants.DRIVETRAIN_MAX_CURRENT);
    rightMaster.setSmartCurrentLimit(Constants.DRIVETRAIN_MAX_CURRENT);
    leftFollower1.setSmartCurrentLimit(Constants.DRIVETRAIN_MAX_CURRENT);
    rightFollower1.setSmartCurrentLimit(Constants.DRIVETRAIN_MAX_CURRENT);
    leftFollower2.setSmartCurrentLimit(Constants.DRIVETRAIN_MAX_CURRENT);
    rightFollower2.setSmartCurrentLimit(Constants.DRIVETRAIN_MAX_CURRENT);

    leftEncoder = leftMaster.getEncoder();
    rightEncoder = rightMaster.getEncoder();
    resetEncoders();

    drive = new DifferentialDrive(leftMotors, rightMotors);
    drive.setDeadband(Constants.DRIVING_DEADBANDS);
    drive.setSafetyEnabled(false);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

    gyro = new ADIS16470_IMU();

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    setConversionFactors();

    inverseMode = false;

    lockedOnTarget = false;

    prevLeftWheelSpeed = 0.0;
    prevRightWheelSpeed = 0.0;
    prevPIDUpdateTime = -1;

    leftDrivePIDController = leftMaster.getPIDController();
    leftDrivePIDController.setP(Constants.kPDriveVel);
    rightDrivePIDController = rightMaster.getPIDController();
    rightDrivePIDController.setP(Constants.kPDriveVel);

    leftDriveFF = new SimpleMotorFeedforward(Constants.ksVoltsLeft,
    Constants.kvVoltSecondsPerMeterLeft,
    Constants.kaVoltSecondsSquaredPerMeterLeft);

    rightDriveFF = new SimpleMotorFeedforward(Constants.ksVoltsRight,
    Constants.kvVoltSecondsPerMeterRight,
    Constants.kaVoltSecondsSquaredPerMeterRight);

    turnToAnglePIDController = new PIDController(Constants.kTurnToAngleP, Constants.kTurnToAngleI, Constants.kTurnToAngleD);
    // Set the controller to be continuous (because it is an angle controller)
    turnToAnglePIDController.enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    turnToAnglePIDController.setTolerance(Constants.kTurnToAngleToleranceDeg, Constants.kTurnToAngleRateToleranceDegPerS);

  }

  @Override
  public void periodic() {
    odometry.update(getHeadingAsRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    if(Constants.USE_LOGGING){
      updateLogs.updateDrivetrainLogData();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static Drivetrain getInstance() {
    if (drivetrain == null) {
      drivetrain = new Drivetrain();
    }
    return drivetrain;
  }

  private void setConversionFactors() {
    leftEncoder.setPositionConversionFactor(Constants.DRIVE_ENC_ROT_TO_DIST);
    rightEncoder.setPositionConversionFactor(Constants.DRIVE_ENC_ROT_TO_DIST);
    leftEncoder.setVelocityConversionFactor(Constants.DRIVE_ENC_ROT_TO_DIST / 60.0);
    rightEncoder.setVelocityConversionFactor(Constants.DRIVE_ENC_ROT_TO_DIST / 60.0);
  }

  // Returns the current wheel speeds
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  public void updateDrivetrainInfoOnDashboard() {
    SmartDashboard.putNumber("Heading", getHeading());
    SmartDashboard.putNumber("Odometry X", odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Odometry Y", odometry.getPoseMeters().getTranslation().getY());
    SmartDashboard.putNumber("Odometry Pose", getPoseHeading());
    SmartDashboard.putNumber("Left drive current", leftMaster.getOutputCurrent());
    SmartDashboard.putNumber("Right drive current", rightMaster.getOutputCurrent());
    SmartDashboard.putNumber("Left drive current", leftMaster.getOutputCurrent());
    SmartDashboard.putNumber("Right drive current", rightMaster.getOutputCurrent());
    
    //only heading and odometry in competition mode
    if(Constants.OI_CONFIG != OIConfig.COMPETITION){
      SmartDashboard.putBoolean("Locked on target", isLockedOnTarget());  

      SmartDashboard.putNumber("LDrive enc pos", getLeftEncoderPosition());
      SmartDashboard.putNumber("RDrive enc pos", getRightEncoderPosition());
      SmartDashboard.putNumber("LDrive enc vel", getLeftEncoderVelocity());
      SmartDashboard.putNumber("RDrive enc vel", getRightEncoderVelocity());
      SmartDashboard.putNumber("Left wheel speeds", getWheelSpeeds().leftMetersPerSecond);
      SmartDashboard.putNumber("Right wheel speeds", getWheelSpeeds().rightMetersPerSecond);
    }

  }

  public void setBrake() {
    leftMaster.setIdleMode(IdleMode.kBrake);
    rightMaster.setIdleMode(IdleMode.kBrake);

    leftFollower1.setIdleMode(IdleMode.kBrake);
    rightFollower1.setIdleMode(IdleMode.kBrake);

    leftFollower2.setIdleMode(IdleMode.kBrake);
    rightFollower2.setIdleMode(IdleMode.kBrake);
  }

  public void setCoast() {
    leftMaster.setIdleMode(IdleMode.kCoast);
    rightMaster.setIdleMode(IdleMode.kCoast);

    leftFollower1.setIdleMode(IdleMode.kCoast);
    rightFollower1.setIdleMode(IdleMode.kCoast);

    leftFollower2.setIdleMode(IdleMode.kCoast);
    rightFollower2.setIdleMode(IdleMode.kCoast);
  }
  
  public void resetEncoders() {
    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);
  }

  public void arcadeDrive(double speed, double turn) {
    drive.arcadeDrive(speed * Constants.SPEED_MULTIPLIER, -turn * Constants.TURN_MULTIPLIER,
        Constants.DRIVE_USE_SQUARED_INPUTS);
  }

  public void curvatureDrive(double speed, double turn) {
    drive.curvatureDrive(speed * Constants.SPEED_MULTIPLIER, turn * Constants.TURN_MULTIPLIER,
        true);
  }

  public double getHeading() {
    headingValue = gyro.getAngle();
    return Math.IEEEremainder(headingValue, 360);
  }

  public Rotation2d getHeadingAsRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public double getUnboundedHeading() {
    return gyro.getAngle();
  }

  public void resetPose(Pose2d estimatedPostition, Rotation2d gyroAngle) {
    resetEncoders();
    odometry.resetPosition(estimatedPostition, gyroAngle);
  }

  public void calibrateGyro() {
    gyro.calibrate();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public double getPoseHeading(){
    return odometry.getPoseMeters().getRotation().getDegrees();
  }

  public void resetGyro() {
    gyro.reset();
    resetPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)), new Rotation2d(0.0));
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    drive.feed();
  }

  //Encoder Getters
  public double getLeftEncoderPosition(){
    return leftEncoder.getPosition();
  }

  public double getRightEncoderPosition(){
    return rightEncoder.getPosition();
  }

  public double getLeftEncoderVelocity(){
    return leftEncoder.getVelocity();
  }

  public double getRightEncoderVelocity(){
    return rightEncoder.getVelocity();
  }

  public double getAverageEncoderDistance(){
    return (leftEncoder.getPosition() + rightEncoder.getPosition())/2.0;
  }

  public double getAverageEncoderVelocity(){
    return (leftEncoder.getVelocity() + rightEncoder.getVelocity())/2.0;
  }


  //Setpoint Getters
  public double getSpeedSetpoint(){
    return speedSetpoint;
  }
  public double getTurnSetpoint(){
    return turnSetpoint;
  }

  public double getLeftMasterVelocity(){
    return leftMaster.get();
  }
  
  public double getLeftFollowerVelocity(){
    return leftFollower1.get();
  }

  public double getLeftFollower2Velocity(){
    return leftFollower2.get();
  }
  
  public double getRightMasterVelocity(){
    return rightMaster.get();
  }
  
  public double getRightFollowerVelocity(){
    return rightFollower1.get();
  }

  public double getRightFollower2Velocity(){
    return rightFollower2.get();
  }


  //Current Getters
  public double getLeftMasterCurrent(){
    return leftMaster.getOutputCurrent();
  }
  
  public double getLeftFollowerCurrent(){
    return leftFollower1.getOutputCurrent();
  }

  public double getLeftFollower2Current(){
    return leftFollower2.getOutputCurrent();
  }
  
  public double getRightMasterCurrent(){
    return rightMaster.getOutputCurrent();
  }
  
  public double getRightFollowerCurrent(){
    return rightFollower1.getOutputCurrent();
  }

  public double getRightFollower2Current(){
    return rightFollower2.getOutputCurrent();
  }


  //Motor Temperature Getters
  public double getLeftMasterMotorTemperature(){
    return leftMaster.getMotorTemperature();
  }
  
  public double getLeftFollowerMotorTemperature(){
    return leftFollower1.getMotorTemperature();
  }

  public double getLeftFollower2MotorTemperature(){
    return leftFollower2.getMotorTemperature();
  }
  
  public double getRightMasterMotorTemperature(){
    return rightMaster.getMotorTemperature();
  }
  
  public double getRightFollowerMotorTemperature(){
    return rightFollower1.getMotorTemperature();
  }

  public double getRightFollower2MotorTemperature(){
    return rightFollower2.getMotorTemperature();
  }

  public void setToInverseMode(){
    inverseMode = true;
  }

  public void setToRegularMode(){
    inverseMode = false;
  }

  public boolean isInverseMode(){
    return inverseMode;
  }

  public PIDController getTurnPID(){
    return turnToAnglePIDController;
  }

  public boolean isLockedOnTarget(){
    return lockedOnTarget;
  }

  public void setLockedOnTarget(boolean lockedOn){
    lockedOnTarget = lockedOn;
  }

  public void updateDrivePIDControllers(double leftWheelSpeed, double rightWheelSpeed) {
    double dt = Timer.getFPGATimestamp() - prevPIDUpdateTime;
    if(prevPIDUpdateTime != -1){
      leftDrivePIDController.setReference(leftWheelSpeed, CANSparkMax.ControlType.kVelocity, 0, leftDriveFF.calculate(leftWheelSpeed, (leftWheelSpeed-prevLeftWheelSpeed)/dt));
      rightDrivePIDController.setReference(rightWheelSpeed, CANSparkMax.ControlType.kVelocity, 0, rightDriveFF.calculate(rightWheelSpeed, (rightWheelSpeed-prevRightWheelSpeed)/dt));
    }
    else{
      leftDrivePIDController.setReference(leftWheelSpeed, CANSparkMax.ControlType.kVelocity, 0, leftDriveFF.calculate(leftWheelSpeed));
      rightDrivePIDController.setReference(rightWheelSpeed, CANSparkMax.ControlType.kVelocity, 1, rightDriveFF.calculate(rightWheelSpeed));
    }
    prevLeftWheelSpeed = leftWheelSpeed;
    prevRightWheelSpeed = rightWheelSpeed;
    prevPIDUpdateTime = Timer.getFPGATimestamp();
    drive.feed();
  }

}