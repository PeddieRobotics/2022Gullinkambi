package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.utils.Constants;

public class ShootLayup extends CommandBase {

  private Flywheel flywheel;
  private Hopper hopper;
  private Drivetrain drivetrain;

  private boolean isAuto;

  public ShootLayup(boolean autonomous) {
    flywheel = Flywheel.getInstance();
    hopper = Hopper.getInstance();
    drivetrain = Drivetrain.getInstance();
    
    addRequirements(flywheel, hopper, drivetrain);

    isAuto = autonomous;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setBrake();
    drivetrain.arcadeDrive(0, 0);
    flywheel.setHood(true); // no hood for high shot
    flywheel.runFlywheelSetpoint(SmartDashboard.getNumber("Teleop: layup RPM", Constants.FLYWHEEL_RPM_LAYUP));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check whether the speed of flywheel is good enough to shoot
    if (flywheel.isAtRPM(Constants.FLYWHEEL_THRESHOLD_LAYUP)) {
      flywheel.setShooterLock(true);
      hopper.setHopperVelocity(SmartDashboard.getNumber("Teleop: Hopper shoot layup speed", Constants.HOPPER_SHOOT_LAYUP_SPEED));
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.stopHopper();
    flywheel.runFlywheelSetpoint(0);
    flywheel.setShooterLock(false);

    if(!isAuto){
      drivetrain.setCoast();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
