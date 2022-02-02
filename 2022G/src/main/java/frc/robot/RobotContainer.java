// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommands.Drive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final OI oi;
  private final Drivetrain drivetrain;
  // private final Intake intake;
  // private final Hopper hopper;
  // private final Flywheel flywheel;
  // private final Climber climber;
  // private final Limelight limelight;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    oi = OI.getInstance();
    
    drivetrain = Drivetrain.getInstance();
    //intake = Intake.getInstance();
    //hopper = Hopper.getInstance();
    //flywheel = Flywheel.getInstance();
    //climber = Climber.getInstance();
    //limelight  = Limelight.getInstance();
    
    drivetrain.setDefaultCommand(new Drive());
    //intake.register();
    //hopper.register();
    //flywheel.register();
    //climber.register();
    //limelight.register();
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    drivetrain.resetPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)), new Rotation2d(0.0));
    // needs to be merged/fixed from autonomous work in 2022Preseason repo
    return null;
  }

  public void setupSmartDashboard() {
    drivetrain.putSmartDashboardOverrides();
    // intake.putSmartDashboardOverrides();
    // hopper.putSmartDashboardOverrides();
    // limelight.putSmartDashboardOverrides();
    // climber.putSmartDashboardOverrides();
    // flywheel.putSmartDashboardOverrides();
  }

  // Overrides for interfacing with robot hardware
  // The SmartDashboard fields for all of these should be configured with the putSmartDashboardOverrides method
  // in each respective subsystem.
  public void testAllSystems() {
    // Drivetrain
    // Be exceptionally careful driving the robot via dashboard. Usually done on blocks.
    drivetrain.arcadeDrive(SmartDashboard.getNumber("OR: Drivetrain speed", 0), SmartDashboard.getNumber("OR: Drivetrain turn", 0));
    
    // Intake
    // intake.runIntake(SmartDashboard.getNumber("OR: Intake speed", 0), SmartDashboard.getBoolean("Intake state", false));

    // Hopper
    // hopper.runHopper(SmartDashboard.getNumber("OR: Hopper speed", 0));

    // Flywheel
    // flywheel.setHood(SmartDashboard.getBoolean("OR: Hood up", false));
    // flywheel.runFlywheelSetPoint(SmartDashboard.getNumber("OR: Flywheel setpoint", 0));

    // Climber
    // climber.setClimberSpeed(SmartDashboard.getNumber("OR: Climber speed", 0));
    // climber.setClimberTilt(SmartDashboard.getBoolean("OR: Climber tilt", false));
    // climber.setClimberHook(SmartDashboard.getBoolean("OR: Climber hook", false));


    // Limelight - currently none
  }

  public void setDrivetrainToCoastMode(){
    drivetrain.setCoast();
  }

  public void resetGyro() {
    drivetrain.resetGyro();
  }

  public void setDrivetrainToBrakeMode() {
    drivetrain.setBrake();
  }
}
