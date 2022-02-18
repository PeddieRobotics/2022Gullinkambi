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
  private double FF = Constants.LL_FF;
  private double angle_bound = Constants.LL_ANGLE_BOUND;
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
     
     Kp = SmartDashboard.getNumber("LL KP", Constants.LL_P);
     Ki = SmartDashboard.getNumber("LL KI", Constants.LL_I);
     Kd = SmartDashboard.getNumber("LL KD", Constants.LL_D);
     FF = SmartDashboard.getNumber("LL FF", Constants.LL_FF);
     angle_bound = SmartDashboard.getNumber("LL ANGLE BOUND", Constants.LL_ANGLE_BOUND);
     LL = new PIDController(Kp, Ki, Kd);

     if (limelight.hasTarget()){
        error = limelight.getTx();
        average_error = limelight.getTxAverage();
        if (error> angle_bound){
          steering_adjust = LL.calculate(average_error) + FF;
        }
        else if (error < -angle_bound){
          steering_adjust = LL.calculate(average_error) - FF;
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
  }

  @Override
  public boolean isFinished() { 
    /*if(error>-angle_bound && error<angle_bound){
      return true;
    }
    else return false;*/
    return false;
}
}