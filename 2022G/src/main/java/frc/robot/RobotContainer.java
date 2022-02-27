// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ClimbCommands.ExtendArm;
import frc.robot.commands.ClimbCommands.InitializeArm;
import frc.robot.commands.ClimbCommands.RetractArm;
import frc.robot.commands.DriveCommands.Drive;
import frc.robot.commands.IntakeCommands.IndexCargo;
import frc.robot.commands.ShootCommands.RevUpFlywheel;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.Constants;
import frc.robot.subsystems.Lights;
import frc.robot.OI;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final OI oi;
  private final Drivetrain drivetrain;
  private final Autonomous autonomous;
  private final Intake intake;
  private final Lights lights;
  private final Hopper hopper;
  private final Flywheel flywheel;
  private final Climber climber;
  public final Limelight limelight;  //SWITCH BACK TO PRIVATE AFTER TESTING PID

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    drivetrain = Drivetrain.getInstance();
    intake = Intake.getInstance();
    hopper = Hopper.getInstance();
    flywheel = Flywheel.getInstance();
    climber = Climber.getInstance();
    limelight = Limelight.getInstance();
    oi = OI.getInstance();
    lights = Lights.getInstance();
    autonomous = Autonomous.getInstance();

    drivetrain.setDefaultCommand(new Drive());
    intake.register();
    hopper.setDefaultCommand(new IndexCargo());
    flywheel.setDefaultCommand(new RevUpFlywheel());
    climber.register();
    limelight.register();

    setupSmartDashboard();

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    drivetrain.resetPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)), new Rotation2d(0.0));
    return autonomous.returnAutonomousCommand();
  }

  public void setupSmartDashboard() {
    SmartDashboard.putNumber("Teleop: shootLL RPM delta", 0);
    intake.putSmartDashboardOverrides();
    hopper.putSmartDashboardOverrides();
    limelight.putSmartDashboardOverrides();
    climber.putSmartDashboardOverrides();
    flywheel.putSmartDashboardOverrides();
    SmartDashboard.putString("Mode", Constants.OI_CONFIG.toString());
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  // Overrides for interfacing with robot hardware
  // The SmartDashboard fields for all of these should be configured with the
  // putSmartDashboardOverrides method
  // in each respective subsystem.

  public void testAllSystems() {
    intake.updateIntakeFromDashboard();
    hopper.updateHopperFromDashboard();
    flywheel.updateFlywheelFromDashboard();
    climber.updateClimberFromDashboard();
  }

  public void updateInfoOnDashboard(){
    SmartDashboard.putNumber("Remaining Match Time", Timer.getMatchTime());
    drivetrain.updateDrivetrainInfoOnDashboard();
    intake.updateIntakeInfoOnDashboard();
    hopper.updateHopperInfoOnDashboard();
    flywheel.updateFlywheelInfoOnDashboard();
    climber.updateClimberInfoOnDashboard();
    limelight.updateLimelightInfoOnDashboard();
    SmartDashboard.putNumber("Angle", drivetrain.getHeading());
  }

  public void stopAllSystems(){
    intake.stopIntake();
    hopper.stopHopper();
    flywheel.stopFlywheel();
  }

  public void setDrivetrainToCoastMode() {
    drivetrain.setCoast();
  }

  public void resetGyro() {
    drivetrain.resetGyro();
  }

  public void calibrateGyro() {
    drivetrain.calibrateGyro();
  }

  public void setDrivetrainToBrakeMode() {
    drivetrain.setBrake();
  }

}
