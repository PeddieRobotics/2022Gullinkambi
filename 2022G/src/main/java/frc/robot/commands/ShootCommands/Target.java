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
  private double steering_adjust;
  private double error;
  private double average_error;
  private double Kp = Constants.LL_P;
  private double Ki = Constants.LL_I;
  private double Kd = Constants.LL_D;
  private double min_command = 0.35;
  private double angle_bound = 1;
  private PIDController LL;

  public Target() {
    limelight = Limelight.getInstance();
    drivetrain = Drivetrain.getInstance();
    addRequirements(drivetrain);
  }
  @Override
  public void initialize() {}

  @Override
  public void execute() {
     LL = new PIDController(Kp, Ki, Kd);
     System.out.println(limelight.getTx());
     if (limelight.hasTarget()){
        error = limelight.getTx();
        average_error = limelight.getTxAverage();
        if (error> angle_bound){
          steering_adjust = LL.calculate(average_error) + min_command;
        }
        else if (error < -angle_bound){
          steering_adjust = LL.calculate(average_error) - min_command;
        }
        else{
          steering_adjust = 0;
        }  
      } 
     else{
       steering_adjust=0;
     }  
      drivetrain.arcadeDrive(0, steering_adjust);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0,0);
  }

  @Override
  public boolean isFinished() { 
    if(error>-angle_bound && error<angle_bound){
      return true;
    }
    else return false;
}
}