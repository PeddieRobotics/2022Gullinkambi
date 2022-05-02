// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ClimbCommands.InitializeArm;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Lights;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final int kMotorPort = 0;
  private static final int kEncoderAChannel = 0;
  private static final int kEncoderBChannel = 1;

  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private Lights lights;

  private UsbCamera intakeCamera;
  private UsbCamera climberArmCamera;
  private Flywheel flywheel;

  private final DCMotor flywheelGearbox = DCMotor.getNEO(2);
  private final Encoder encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);
  private final FlywheelSim flywheelSim = new FlywheelSim(flywheelGearbox, 1.0, 0.158151833);
  private final EncoderSim encoderSim = new EncoderSim(encoder);
  private final CANSparkMax canSparkMaxMotor = new CANSparkMax(kMotorPort, MotorType.kBrushless);
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    robotContainer.setDrivetrainToCoastMode();
    robotContainer.calibrateGyro();
    robotContainer.setupSmartDashboard();
    lights = Lights.getInstance();

    //Camera
<<<<<<< HEAD
    /*intakeCamera = CameraServer.startAutomaticCapture("USBCamera_Intake", 0);
    intakeCamera.setExposureAuto();
    intakeCamera.setFPS(15);
    intakeCamera.setResolution(320,180);
    intakeCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
=======
    // intakeCamera = CameraServer.startAutomaticCapture("USBCamera_Intake", 0);
    // intakeCamera.setExposureAuto();
    // intakeCamera.setFPS(15);
    // intakeCamera.setResolution(320,180);
    // intakeCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
>>>>>>> c4af2c093887d22bb8893efee7d619845ad6f687

    // climberArmCamera = CameraServer.startAutomaticCapture("USBCamera_Arm", 1); 
    // climberArmCamera.setExposureAuto();
    // climberArmCamera.setFPS(15);
    // climberArmCamera.setResolution(480, 320);
    // climberArmCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

<<<<<<< HEAD
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);
    */
=======
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);
>>>>>>> c4af2c093887d22bb8893efee7d619845ad6f687
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    robotContainer.updateInfoOnDashboard();
  }

  @Override
  public void simulationPeriodic() {
  flywheelSim.setInput(canSparkMaxMotor.get() * RobotController.getBatteryVoltage());
  //Update with a loop, standard loop time is 20ms
  flywheelSim.update(0.020);
  //flywheelSIm.setInput(), this is where I left off, to continue tomorrow, -Anthony
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    robotContainer.resetGyro();
    robotContainer.setDrivetrainToCoastMode();
    robotContainer.stopAllSystems();
    robotContainer.setupSmartDashboard();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    robotContainer.resetGyro();
    robotContainer.setDrivetrainToBrakeMode();

    // schedule the autonomous command (example)
    if (!(robotContainer.getAutonomousCommand() == null)) {
      autonomousCommand = robotContainer.getAutonomousCommand();
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    robotContainer.setDrivetrainToCoastMode();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    CommandScheduler.getInstance().schedule(new InitializeArm());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    robotContainer.testAllSystems();
  }
}
