package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.utils.Constants;

public class ShootFar extends CommandBase {

  private Flywheel flywheel;
  private Hopper hopper;

  public ShootFar() {
    flywheel = Flywheel.getInstance();
    hopper = Hopper.getInstance();
    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheel.setHood(true); // turn hood on for shoot far with high speed
    flywheel.setShooterLock(true);

    flywheel.runFlywheelSetpoint(Constants.FLYWHEEL_RPM_FAR);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check whether the speed of flywheel is good enough to shoot
    if (flywheel.isAtRPM(Constants.FLYWHEEL_THRESHOLD_FAR)) {
      hopper.runHopper(Constants.HOPPER_SPEED);
    } else {
      hopper.runHopper(0.0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("SHOOT FAR INTERRUPTED");
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
