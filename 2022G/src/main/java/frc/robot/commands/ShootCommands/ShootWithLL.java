package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.Constants;

public class ShootWithLL extends CommandBase {

  private Flywheel flywheel;
  private Hopper hopper;
  private Drivetrain drivetrain;
  private Limelight limelight;
  private double rpm;
  private boolean isAuto;

  public ShootWithLL(boolean autonomous) {
    flywheel = Flywheel.getInstance();
    hopper = Hopper.getInstance();
    drivetrain = Drivetrain.getInstance();
    limelight = Limelight.getInstance();
    addRequirements(flywheel, hopper);

    isAuto = autonomous;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setBrake();
    rpm = Constants.DIST_TO_RPM.get(limelight.getDistance());
    flywheel.setHood(true); // turn hood on for LL shot
    flywheel.runFlywheelSetpoint(rpm + SmartDashboard.getNumber("Teleop: shootLL RPM delta", 0));
    flywheel.setShooterLock(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(flywheel.getPrimaryFlywheelCurrent() > 20){
      System.out.println("SHOT at" + Timer.getFPGATimestamp());
    }
    // Check whether the speed of flywheel is good enough to shoot
    if (flywheel.isAtRPM(Constants.FLYWHEEL_THRESHOLD_SHOOTLL)) {
      hopper.setHopperVelocity(SmartDashboard.getNumber("Teleop: Hopper shoot speed", Constants.HOPPER_SHOOT_SPEED));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setLockedOnTarget(false);
    flywheel.setShooterLock(false);
    if(!isAuto){
      hopper.stopHopper();
      flywheel.runFlywheelSetpoint(0);
      flywheel.setHood(false);
      drivetrain.setCoast();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
