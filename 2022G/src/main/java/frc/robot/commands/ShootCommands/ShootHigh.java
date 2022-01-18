package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Flywheel;
import frc.robot.utils.Constants;

public class ShootLayup extends CommandBase {

  private Shooter shooter;

  public ShootLayup() {
    shooter = Shooter.getInstance();
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setHood(false);
    shooter.setFlywheelVelocity(SmartDashboard.getNumber("ShootLayup", Constants.RPM_LAYUP));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Checking whether the speed of flywheel is good enough to shoot
    if (shooter.isAtRPM(Constants.SHOOTER_THRESHOLD_LAYUP)) {
      shooter.runTowerBelt(0.8);
      DriverStation.reportError("firing upper", false);
    } else {
      shooter.runTowerBelt(0.0);
      DriverStation.reportError("stopping upper", false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
