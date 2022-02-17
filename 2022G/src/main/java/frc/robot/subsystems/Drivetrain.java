// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.*;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain drivetrain;

  private final CANSparkMax leftMaster, rightMaster, leftFollower1, rightFollower1;
  private CANSparkMax rightFollower2,leftFollower2;
  private final MotorControllerGroup leftMotors, rightMotors;

  private final DifferentialDrive drive;
  private final DifferentialDriveOdometry odometry;
  private final ADIS16470_IMU gyro;

  private double headingValue;

  private final RelativeEncoder leftEncoder, rightEncoder;

  NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");

  //Logging
  private static UpdateLogs updateLogs = UpdateLogs.getInstance();

  private double speedSetpoint, turnSetpoint;

  public Drivetrain() {
    if(Constants.IS_GULLINKAMBI){
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
    } else {
      leftMaster = new CANSparkMax(RobotMapMini.MOTOR_DRIVE_LEFT_MASTER, MotorType.kBrushless);
      rightMaster = new CANSparkMax(RobotMapMini.MOTOR_DRIVE_RIGHT_MASTER, MotorType.kBrushless);
      leftFollower1 = new CANSparkMax(RobotMapMini.MOTOR_DRIVE_LEFT_FOLLOWER1, MotorType.kBrushless);
      rightFollower1 = new CANSparkMax(RobotMapMini.MOTOR_DRIVE_RIGHT_FOLLOWER1, MotorType.kBrushless);

      leftMotors = new MotorControllerGroup(leftMaster, leftFollower1);
      rightMotors = new MotorControllerGroup(rightMaster, rightFollower1);
      
      leftFollower1.follow(leftMaster);
      rightFollower1.follow(rightMaster);

      leftMaster.setSmartCurrentLimit(Constants.DRIVETRAIN_MAX_CURRENT);
      rightMaster.setSmartCurrentLimit(Constants.DRIVETRAIN_MAX_CURRENT);
      leftFollower1.setSmartCurrentLimit(Constants.DRIVETRAIN_MAX_CURRENT);
      rightFollower1.setSmartCurrentLimit(Constants.DRIVETRAIN_MAX_CURRENT);
    }

    leftEncoder = leftMaster.getEncoder();
    rightEncoder = rightMaster.getEncoder();
    resetEncoders();


    drive = new DifferentialDrive(leftMotors, rightMotors);
    drive.setDeadband(Constants.DRIVING_DEADBANDS);
    drive.setSafetyEnabled(false);

    leftMaster.setInverted(true);
    rightMaster.setInverted(false);

    gyro = new ADIS16470_IMU();
    //calibrateGyro();
    //gyro.reset();

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    setConversionFactors();
  }

  @Override
  public void periodic() {
    odometry.update(getHeadingAsRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

    updateLogs.updateDrivetrainLogData();
    var translation = odometry.getPoseMeters().getTranslation();
    m_xEntry.setNumber(translation.getX());
    m_yEntry.setNumber(translation.getY());
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
    SmartDashboard.putNumber("L enc pos", getLeftEncoderPosition());
    SmartDashboard.putNumber("R enc pos", getRightEncoderPosition());
    SmartDashboard.putNumber("L enc vel", getLeftEncoderVelocity());
    SmartDashboard.putNumber("R enc vel", getRightEncoderVelocity());
    SmartDashboard.putNumber("Heading", getHeading());
    SmartDashboard.putNumber("Unbounded Heading", getUnboundedHeading());
    SmartDashboard.putNumber("Odometry X", odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Odometry Y", odometry.getPoseMeters().getTranslation().getY());
    SmartDashboard.putNumber("Average Velocity", getAverageEncoderVelocity());
    SmartDashboard.putNumber("Left Encoder Velocity", getLeftEncoderVelocity());
    SmartDashboard.putNumber("Right Encoder Velocity", getRightEncoderVelocity());

  }

  public void setBrake() {

    leftMaster.setIdleMode(IdleMode.kBrake);
    rightMaster.setIdleMode(IdleMode.kBrake);

    leftFollower1.setIdleMode(IdleMode.kBrake);
    rightFollower1.setIdleMode(IdleMode.kBrake);

    // With Gullinkambi
    if(Constants.IS_GULLINKAMBI){
      leftFollower2.setIdleMode(IdleMode.kBrake);
      rightFollower2.setIdleMode(IdleMode.kBrake);
    }
  }

  public void setCoast() {
    leftMaster.setIdleMode(IdleMode.kCoast);
    rightMaster.setIdleMode(IdleMode.kCoast);

    leftFollower1.setIdleMode(IdleMode.kCoast);
    rightFollower1.setIdleMode(IdleMode.kCoast);

    // With Gullinkambi
    if(Constants.IS_GULLINKAMBI){
      leftFollower2.setIdleMode(IdleMode.kCoast);
      rightFollower2.setIdleMode(IdleMode.kCoast);
    }
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);
  }

  public void arcadeDrive(double speed, double turn) {
    drive.arcadeDrive(speed * Constants.SPEED_MULTIPLIER, turn * Constants.TURN_MULTIPLIER,
        Constants.DRIVE_USE_SQUARED_INPUTS);
  }

  public void curvatureDrive(double speed, double turn) {
    drive.curvatureDrive(speed * Constants.SPEED_MULTIPLIER, turn * Constants.TURN_MULTIPLIER,
        true);
  }

  public void putSmartDashboardOverrides() {
    SmartDashboard.putNumber("OR: Drivetrain speed", 0);
    SmartDashboard.putNumber("OR: Drivetrain turn", 0);
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

  public void resetGyro() {
    gyro.reset();
    resetPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)), new Rotation2d(0.0));
    resetEncoders();
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
    return -rightEncoder.getPosition();
  }

  public double getLeftEncoderVelocity(){
    return leftEncoder.getVelocity();
  }

  public double getRightEncoderVelocity(){
    return -rightEncoder.getVelocity();
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


  //Motor Temperaure Getters
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
}