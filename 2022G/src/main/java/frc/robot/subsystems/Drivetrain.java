// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.RobotMap;

import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain drivetrain;

  private final CANSparkMax leftMaster, rightMaster, leftFollower1, rightFollower1, leftFollower2, rightFollower2;
  private final MotorControllerGroup leftMotors, rightMotors;

  private final DifferentialDrive drive;
  private final DifferentialDriveOdometry odometry;
  private final AHRS gyro;

  private double headingValue;

  private final RelativeEncoder leftEncoder, rightEncoder;

  public Drivetrain() {
    leftMaster = new CANSparkMax(RobotMap.MOTOR_DRIVE_LEFT_MASTER, MotorType.kBrushless);
    rightMaster = new CANSparkMax(RobotMap.MOTOR_DRIVE_RIGHT_MASTER, MotorType.kBrushless);
    leftFollower1 = new CANSparkMax(RobotMap.MOTOR_DRIVE_LEFT_FOLLOWER1, MotorType.kBrushless);
    rightFollower1 = new CANSparkMax(RobotMap.MOTOR_DRIVE_RIGHT_FOLLOWER1, MotorType.kBrushless);
    leftFollower2 = new CANSparkMax(RobotMap.MOTOR_DRIVE_LEFT_FOLLOWER2, MotorType.kBrushless);
    rightFollower2 = new CANSparkMax(RobotMap.MOTOR_DRIVE_RIGHT_FOLLOWER2, MotorType.kBrushless);

    leftMotors = new MotorControllerGroup(leftMaster, leftFollower1, leftFollower2);
    rightMotors = new MotorControllerGroup(rightMaster, rightFollower1, rightFollower2);
    
    drive = new DifferentialDrive(leftMotors, rightMotors);
    drive.setDeadband(Constants.DRIVING_DEADBANDS);
    drive.setSafetyEnabled(false);

    leftMaster.setInverted(true);
    rightMaster.setInverted(false);
    
    leftFollower1.follow(leftMaster);
    leftFollower2.follow(leftMaster);
    rightFollower1.follow(rightMaster);
    rightFollower2.follow(rightMaster);

    leftEncoder = leftMaster.getEncoder();
    rightEncoder = rightMaster.getEncoder();

    gyro = new AHRS();

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

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

public void putSmartDashboard(){
  SmartDashboard.putNumber("L enc pos", getLeftEncoderPosition());
  SmartDashboard.putNumber("R enc pos", getRightEncoderPosition());
  SmartDashboard.putNumber("L enc vel", getLeftEncoderVelocity());
  SmartDashboard.putNumber("R enc vel", getRightEncoderVelocity());
  SmartDashboard.putNumber("Heading", getHeading());
}

public void setBrake() {

  leftMaster.setIdleMode(IdleMode.kBrake);
  rightMaster.setIdleMode(IdleMode.kBrake);

  leftFollower1.setIdleMode(IdleMode.kBrake);
  rightFollower1.setIdleMode(IdleMode.kBrake);

  leftFollower2.setIdleMode(IdleMode.kBrake);
  rightFollower2.setIdleMode(IdleMode.kBrake);
}

public void setCoast(){
  leftMaster.setIdleMode(IdleMode.kCoast);
  rightMaster.setIdleMode(IdleMode.kCoast);

  leftFollower1.setIdleMode(IdleMode.kCoast);
  rightFollower1.setIdleMode(IdleMode.kCoast);

  leftFollower2.setIdleMode(IdleMode.kCoast);
  rightFollower2.setIdleMode(IdleMode.kCoast);

}

public void resetEncoders(){
  leftEncoder.setPosition(0.0);
  rightEncoder.setPosition(0.0);
}

public void arcadeDrive(double speed, double turn){
  drive.arcadeDrive(speed, turn,
    Constants.DRIVE_USE_SQUARED_INPUTS);
  }

public void putSmartDashboardOverrides(){
    SmartDashboard.putNumber("OR: Drivetrain speed", 0);
    SmartDashboard.putNumber("OR: Drivetrain turn", 0);
  }

public double getHeading(){
   headingValue = gyro.getAngle();
  return Math.IEEEremainder(headingValue, 360);
  }
}