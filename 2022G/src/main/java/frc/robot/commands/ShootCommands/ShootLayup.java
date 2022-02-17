package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.utils.Constants;

public class ShootLayup extends CommandBase {

  private Flywheel flywheel;
  private Hopper hopper;

  public ShootLayup() {
    flywheel = Flywheel.getInstance();
    hopper = Hopper.getInstance();
    addRequirements(flywheel, hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheel.setHood(false); // no hood for high shot
    flywheel.setShooterLock(true);

    flywheel.runFlywheelSetpoint(SmartDashboard.getNumber("Teleop: Flywheel shoot layup RPM", Constants.FLYWHEEL_RPM_LAYUP));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check whether the speed of flywheel is good enough to shoot
    if (flywheel.isAtRPM(SmartDashboard.getNumber("Teleop: Flywheel shoot low threshold", Constants.FLYWHEEL_THRESHOLD_LOW))){
      hopper.runHopper(SmartDashboard.getNumber("Teleop: Hopper speed", Constants.HOPPER_SPEED));
    } else {
      hopper.stopHopper();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.stopHopper();
    flywheel.stopFlywheel();
    flywheel.setShooterLock(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
