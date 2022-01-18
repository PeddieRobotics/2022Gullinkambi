// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Tower;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.Drive;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final OI oi;
  private final Drivetrain drivetrain;
  private final Intake intake;
  private final Hopper hopper;
  private final Tower tower;
  private final Limelight limelight;
  private final Flywheel flywheel;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    oi = new OI();
    drivetrain = Drivetrain.getInstance();
    intake = Intake.getInstance();
    hopper = Hopper.getInstance();
    tower = Tower.getInstance();
    limelight  = Limelight.getInstance();
    flywheel = Flywheel.getInstance();

    drivetrain.setDefaultCommand(new Drive());
    intake.register();
    hopper.register();
    tower.register();
    limelight.register();
    flywheel.register();
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

  public void setupSmartDashboard() {
    drivetrain.putSmartDashboard();
    intake.putSmartDashboard();
    hopper.putSmartDashboard();
    tower.putSmartDashboard();
    limelight.putSmartDashboard();
    flywheel.putSmartDashboard();
  }

  public void testAllSystems() {
    //Drivetrain
    drivetrain.arcadeDrive(SmartDashboard.getNumber("speed", 0), SmartDashboard.getNumber("turn", 0));
    
    //Intake
    intake.runIntake(SmartDashboard.getNumber("Intake Rollers Speed", 0), SmartDashboard.getBoolean("Intake State", false));

    //Hopper
    hopper.runHopper(SmartDashboard.getNumber("Hopper Left Roller Speed", 0), SmartDashboard.getNumber("Hopper Right Roller Speed", 0), SmartDashboard.getNumber("Hopper Belt Speed", 0));

    //Tower
    tower.runTower(SmartDashboard.getNumber("Tower Lower Belt Speed", 0), SmartDashboard.getNumber("Tower Upper Belt Speed", 0));

    //Flywheel
    flywheel.setHood(SmartDashboard.getBoolean("Hood Up", false));
    flywheel.runFlywheelSetPoint(SmartDashboard.getNumber("Flywheel Setpoint", 0));
  }
}
