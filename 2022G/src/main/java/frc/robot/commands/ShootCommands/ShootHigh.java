package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Flywheel;
import frc.robot.utils.Constants;

public class ShootHigh extends CommandBase {

  private Flywheel flywheel;
    private Tower tower;

  public ShootHigh() {
    flywheel = Flywheel.getInstance();
    tower = Tower.getInstance();
    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheel.setHood(false); //turn hood off cus its a high shot to the high goal
    flywheel.runFlywheelSetPoint(Constants.RPM_FAR);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Checking whether the speed of flywheel is good enough to shoot
    if (flywheel.isAtRPM(Constants.FLYWHEEL_THRESHOLD_HIGH)){
      tower.runUpperBelt(0.8);
  }
  else {
      tower.runLowerBelt(0.0);
  }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tower.stopTower();
    flywheel.stopFlywheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
