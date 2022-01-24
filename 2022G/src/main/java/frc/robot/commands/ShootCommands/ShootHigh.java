package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.utils.Constants;

public class ShootHigh extends CommandBase {

  private Flywheel flywheel;
  private Hopper hopper;

  public ShootHigh() {
    flywheel = Flywheel.getInstance();
    hopper = Hopper.getInstance();
    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheel.setHood(true); // hood for high shot
    flywheel.setShooterLock(true);

    flywheel.runFlywheelSetPoint(Constants.FLYWHEEL_RPM_HIGH);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check whether the speed of flywheel is good enough to shoot
    if (flywheel.isAtRPM(Constants.FLYWHEEL_THRESHOLD_FAR)){
      hopper.runHopper(0.8);
    }
    else {
      hopper.runHopper(0.0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.stopHopper();
    flywheel.stopFlywheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

