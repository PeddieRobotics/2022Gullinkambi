// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.littletonrobotics.junction.inputs.LoggedNetworkTables;
import org.littletonrobotics.junction.io.ByteLogReceiver;
import org.littletonrobotics.junction.io.LogSocketServer;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ClimbCommands.InitializeBothArms;
import frc.robot.commands.ClimbCommands.InitializeRightArm;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.Constants;
import frc.robot.utils.LoggingCustoms.LoggedRobotCustom;
import frc.robot.utils.LoggingCustoms.LoggerCustom;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobotCustom {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private Lights lights;

  private UsbCamera driverCamera;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    if(Constants.USE_LOGGING){
      LoggerCustom logger = LoggerCustom.getInstance();
      setUseTiming(true);
      LoggedNetworkTables.getInstance().addTable("/LiveWindow");
      logger.addDataReceiver(new ByteLogReceiver("/home/lvuser/"));
      logger.addDataReceiver(new LogSocketServer(5800));
      logger.start();
    }

    robotContainer = new RobotContainer();
    robotContainer.setDrivetrainToCoastMode();
    robotContainer.calibrateGyro();
    robotContainer.setupSmartDashboard();
    lights = Lights.getInstance();

    //Camera
    driverCamera = CameraServer.startAutomaticCapture("USBCamera", 0);
    driverCamera.setExposureAuto();
    driverCamera.setFPS(10);
    driverCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    driverCamera.setResolution(1280, 720);
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
    CommandScheduler.getInstance().schedule(new InitializeBothArms());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Flywheel.getInstance().updateFlywheelPIDFromDashboard();
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
