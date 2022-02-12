package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.Constants;

public class ShootFar extends CommandBase {

  private Flywheel flywheel;
  private Hopper hopper;
  private Limelight limelight;
  private double RPM;

  public ShootFar() {
    flywheel = Flywheel.getInstance();
    hopper = Hopper.getInstance();
    limelight = Limelight.getInstance();
    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheel.setHood(true); //turn hood on for shoot far with high speed
    flywheel.setShooterLock(true);
    RPM = Constants.DIST_TO_RPM.get(limelight.getDistance());
    flywheel.runFlywheelSetPoint(RPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check whether the speed of flywheel is good enough to shoot
    if (flywheel.isAtRPM(Constants.FLYWHEEL_THRESHOLD_FAR)){
      hopper.runHopper(Constants.HOPPER_SPEED);
    }
    else {
      hopper.stopHopper();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.stopHopper();
    flywheel.stopFlywheel();
    flywheel.setHood(false);
    flywheel.setShooterLock(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

