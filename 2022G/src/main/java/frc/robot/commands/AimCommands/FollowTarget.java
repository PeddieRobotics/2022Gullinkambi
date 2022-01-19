package frc.robot.commands.AimCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Turret.TurretMode;
import frc.robot.subsystems.Limelight;

/** An example command that uses an example subsystem. */
public class FollowTarget extends CommandBase {
    
  private Turret turret;
  private Limelight limelight;

  public FollowTarget() {
    turret = Turret.getInstance();
    limelight = Limelight.getInstance();

    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Adjust the current shoulder angle by a multiple of the y-axis Limelight error
    double newAngle = turret.getCurrentAngle() + turret.getLimelightErrorScaleFactor()*limelight.getTy();
    SmartDashboard.putNumber("New shoulder angle (LL)", newAngle);
    
    if(Math.abs(limelight.getTy()) > turret.getLimelightErrorMaxTol()){
        turret.setTurret(newAngle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(turret.getMode() == TurretMode.INTAKING || turret.getMode() == TurretMode.OVERRIDING){
      return true;
    }
    return false;
  }
}