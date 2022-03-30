package frc.robot.commands.ShootCommands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.Constants;
import frc.robot.utils.RollingAverage;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TargetLLAdjust extends CommandBase {
  private Limelight limelight;
  private Drivetrain drivetrain;
  private Flywheel flywheel;
  private Hopper hopper;

  private double ff;
  private double steering_adjust;
  private double error;
  private double average_error;
  private double angle_bound;
  private PIDController limelightPIDController;
  private boolean isAuto, limelightBroken;

  public TargetLLAdjust(boolean autonomous) {
    limelight = Limelight.getInstance();
    drivetrain = Drivetrain.getInstance();
    flywheel = Flywheel.getInstance();
    hopper = Hopper.getInstance();

    addRequirements(drivetrain, flywheel);

    limelightPIDController = limelight.getPIDController();

    angle_bound = Constants.LL_ANGLE_BOUND;

    isAuto = autonomous;

  }
  @Override
  public void initialize() {
      // Assume by default that we're not locked on a limelight target. Shouldn't be needed, but placed here as a safety on the logic elsewhere.
      if(!limelight.isActive()){
        limelightBroken = true;
      }
  }

  @Override
  public void execute() {
     ff = limelight.getFF();

     if (limelight.hasTarget()){
        // Get the flywheel up to the full speed while targeting
        double rpm = Constants.DIST_TO_RPM.get(limelight.getDistance());
        //flywheel.runFlywheelSetpoint(rpm + SmartDashboard.getNumber("Teleop: shootLL RPM delta", 0));

        error = limelight.getTx();
        average_error = limelight.getTxAverage();
        if (average_error < -angle_bound){
          steering_adjust = limelightPIDController.calculate(average_error) + ff;
        }
        else if (average_error > angle_bound){
          steering_adjust = limelightPIDController.calculate(average_error) - ff;
        }
        else{
          steering_adjust = 0;
        }  
      } 
     else{
       steering_adjust = 0;
     }  
      drivetrain.arcadeDrive(0, steering_adjust);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0,0);

    if(interrupted && !hopper.sensesBallBottom() && !hopper.sensesBallTop()){
      flywheel.runFlywheelSetpoint(0);
    }

  }

  @Override
  public boolean isFinished() { 
    if(limelightBroken){
      return true;
    }

    return (Math.abs(limelight.getTx()) < angle_bound);
  }
}