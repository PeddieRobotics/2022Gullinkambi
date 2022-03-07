package frc.robot.commands.ShootCommands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.Constants;
import frc.robot.utils.RollingAverage;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Target extends CommandBase {
  private final Limelight limelight;
  private final Drivetrain drivetrain;

  private double ff;
  private double steering_adjust;
  private double error;
  private double average_error;
  private double angle_bound;
  private PIDController limelightPIDController;

  public Target() {
    limelight = Limelight.getInstance();
    drivetrain = Drivetrain.getInstance();

    addRequirements(drivetrain);

    limelightPIDController = limelight.getPIDController();

    angle_bound = Constants.LL_ANGLE_BOUND;
  }
  @Override
  public void initialize() {
      // Assume by default that we're not locked on a limelight target. Shouldn't be needed, but placed here as a safety on the logic elsewhere.
      drivetrain.setLockedOnTarget(false);
  }

  @Override
  public void execute() {
     ff = limelight.getFF();

     if (limelight.hasTarget()){
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
    // If we end this command with the LL seeing target AND we weren't interrupted (e.g. trigger release), we are locked to target now
    // Otherwise we must be ending immediately because no target was found for alignment
    if(limelight.hasTarget() && !interrupted){
      drivetrain.setLockedOnTarget(true);
    }
  }

  @Override
  public boolean isFinished() { 
    return (Math.abs(limelight.getTx()) < angle_bound);
}
}