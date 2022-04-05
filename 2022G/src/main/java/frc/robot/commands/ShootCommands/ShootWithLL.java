package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.Constants;

public class ShootWithLL extends CommandBase {

  private Flywheel flywheel;
  private Hopper hopper;
  private Drivetrain drivetrain;
  private Limelight limelight;
  private Lights lights;

  private double rpm;
  private boolean isAuto;

  public ShootWithLL(boolean autonomous) {
    flywheel = Flywheel.getInstance();
    hopper = Hopper.getInstance();
    drivetrain = Drivetrain.getInstance();
    limelight = Limelight.getInstance();
    lights = Lights.getInstance();

    addRequirements(flywheel, hopper);

    isAuto = autonomous;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!SmartDashboard.getBoolean("LL Shot Override", false)){
      rpm = Constants.DIST_TO_RPM.get(limelight.getDistance());
      flywheel.runFlywheelSetpoint(rpm + SmartDashboard.getNumber("Teleop: shootLL RPM delta", 0));
    }
    else{
      flywheel.runFlywheelSetpoint(2550 + SmartDashboard.getNumber("Teleop: shootLL RPM delta", 0));          
    }
    
    drivetrain.setBrake();
    flywheel.setHood(false); // turn hood on for LL shot

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check whether the speed of flywheel is good enough to shoot
    if (!flywheel.getShooterLock() && flywheel.isAtRPM(Constants.FLYWHEEL_THRESHOLD_SHOOTLL)) {
      flywheel.setShooterLock(true);
      double dist = limelight.getDistance();
      if(dist < 100){
        hopper.setHopperVelocity(SmartDashboard.getNumber("Teleop: Hopper shoot LL speed", Constants.HOPPER_SHOOT_LL_SPEED));
      }
      else if(dist >= 100 && dist < 115){
        hopper.setHopperVelocity(SmartDashboard.getNumber("Teleop: Hopper shoot LL speed", Constants.HOPPER_SHOOT_LL_SPEED) + 600);
      }
      else if(dist >= 115 & dist < 130){
        hopper.setHopperVelocity(SmartDashboard.getNumber("Teleop: Hopper shoot LL speed", Constants.HOPPER_SHOOT_LL_SPEED) + 1200);
      }
      else if(limelight.getDistance() >= 130){
        hopper.setHopperVelocity(SmartDashboard.getNumber("Teleop: Hopper shoot LL speed", Constants.HOPPER_SHOOT_LL_SPEED) + 1800);
      }

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
      drivetrain.setCoast();
    }
    else{
      lights.off();
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
