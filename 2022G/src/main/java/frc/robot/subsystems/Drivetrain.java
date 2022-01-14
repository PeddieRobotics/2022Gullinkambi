// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain drivetrain;

  private final CANSparkMax leftMaster, rightMaster, leftFollower, rightFollower;
  private final MotorControllerGroup leftMotors, rightMotors;

  private final DifferentialDrive drive;

  private final RelativeEncoder leftEncoder, rightEncoder;

  
  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {
    leftMaster = new CANSparkMax(RobotMap.DRIVE_LEFT_MASTER, MotorType.kBrushless);
    rightMaster = new CANSparkMax(RobotMap.DRIVE_RIGHT_MASTER, MotorType.kBrushless);
    leftFollower = new CANSparkMax(RobotMap.DRIVE_LEFT_FOLLOWER, MotorType.kBrushless);
    rightFollower = new CANSparkMax(RobotMap.DRIVE_RIGHT_FOLLOWER, MotorType.kBrushless);

    leftMotors = new MotorControllerGroup(leftMaster, leftFollower);
    rightMotors = new MotorControllerGroup(rightMaster, rightFollower);
    
    drive = new DifferentialDrive(leftMotors, rightMotors);
    drive.setDeadband(Constants.DRIVING_DEADBANDS);
    drive.setSafetyEnabled(false);

    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);

    leftEncoder = leftMaster.getEncoder();
    rightEncoder = rightMaster.getEncoder();
  }
  @Override
  public void periodic() {
    putSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

public static Drivetrain getInstance(){
  if(drivetrain == null){
      drivetrain = new Drivetrain();
  }
  return drivetrain;
}

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

// Returns the current wheel speeds
public DifferentialDriveWheelSpeeds getWheelSpeeds() {
  return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
}


public void setBrake() {

  leftMaster.setIdleMode(IdleMode.kBrake);
  rightMaster.setIdleMode(IdleMode.kBrake);

  leftFollower.setIdleMode(IdleMode.kBrake);
  rightFollower.setIdleMode(IdleMode.kBrake);
}

public void setCoast(){
  leftMaster.setIdleMode(IdleMode.kCoast);
  rightMaster.setIdleMode(IdleMode.kCoast);

  leftFollower.setIdleMode(IdleMode.kCoast);
  rightFollower.setIdleMode(IdleMode.kCoast);
}

public void resetEncoders(){
  leftEncoder.setPosition(0.0);
  rightEncoder.setPosition(0.0);
}

public void arcadeDrive(double speed, double turn){
  drive.arcadeDrive(speed, turn,
    Constants.DRIVE_USE_SQUARED_INPUTS);
  }

  public void putSmartDashboard(){
    SmartDashboard.putNumber("speed", 0);
    SmartDashboard.putNumber("turn", 0);
    SmartDashboard.putNumber("L Enc Pos", getLeftEncoderPosition());
    SmartDashboard.putNumber("R Enc Pos", getRightEncoderPosition());
    SmartDashboard.putNumber("L Enc Vel", getLeftEncoderVelocity());
    SmartDashboard.putNumber("R Enc Vel", getRightEncoderVelocity());

  }
}
