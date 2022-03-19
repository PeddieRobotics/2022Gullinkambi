package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.utils.Constants;

public class ShootLow extends CommandBase {

  private Flywheel flywheel;
  private Hopper hopper;
  private double rpm;

  public ShootLow(double rpm) {
    flywheel = Flywheel.getInstance();
    hopper = Hopper.getInstance();
    addRequirements(flywheel, hopper);
    this.rpm = rpm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheel.setHood(true);
    flywheel.setShooterLock(true);
    flywheel.runFlywheelSetpoint(rpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check whether the speed of flywheel is good enough to shoot
    if (flywheel.isAtRPM(Constants.FLYWHEEL_THRESHOLD_LOW)) {
      hopper.setHopperVelocity(SmartDashboard.getNumber("Teleop: Hopper shoot speed", Constants.HOPPER_SHOOT_SPEED));
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.stopHopper();
    flywheel.setHood(false);
    flywheel.runFlywheelSetpoint(0);
    flywheel.setShooterLock(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
