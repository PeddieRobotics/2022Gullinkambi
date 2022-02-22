package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.Constants;

public class ShootWithLL extends CommandBase {

  private Flywheel flywheel;
  private Hopper hopper;
  private Limelight limelight;
  private double rpm;

  public ShootWithLL() {
    flywheel = Flywheel.getInstance();
    hopper = Hopper.getInstance();
    limelight = Limelight.getInstance();
    addRequirements(flywheel, hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rpm = Constants.DIST_TO_RPM.get(limelight.getDistance());
    flywheel.setHood(true); // turn hood on for shoot far with high speed
    flywheel.runFlywheelSetpoint(rpm + SmartDashboard.getNumber("Teleop: shootLL RPM delta", 0)); //!!
    hopper.runHopper(SmartDashboard.getNumber("Teleop: Hopper speed", Constants.HOPPER_SPEED));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check whether the speed of flywheel is good enough to shoot
    if (flywheel.isAtRPM(SmartDashboard.getNumber("Test: shoot LL threshold", Constants.FLYWHEEL_THRESHOLD_SHOOTLL))) {
      flywheel.setShooterLock(true);
    } else {
      flywheel.setShooterLock(false);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.stopHopper();
    flywheel.stopFlywheel();
    flywheel.setHood(false);
    flywheel.setShooterLock(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
