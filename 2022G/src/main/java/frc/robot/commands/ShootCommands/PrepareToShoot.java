package frc.robot.commands.ShootCommands;

import javax.swing.text.DefaultStyledDocument.ElementSpec;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.Constants;

public class PrepareToShoot extends CommandBase {

    private Drivetrain drivetrain;
    private Limelight limelight;
    private Flywheel flywheel;

    private boolean limelightBroken;
  
    public PrepareToShoot() {
        drivetrain = Drivetrain.getInstance();
        limelight = Limelight.getInstance();
        flywheel = Flywheel.getInstance();
        addRequirements(drivetrain, limelight, flywheel);

        limelightBroken = false;
    }
    // Called when the command is initially scheduled
    @Override
    public void initialize() {
      // Set to brake mode
      drivetrain.setBrake();

      // Check if the limelight isn't working, if so we want to end immediately and not try to target
      if(!limelight.isActive()){
        limelightBroken = true;
      }

      // Get horizontal angular error from limelight and calculate new heading
      double currentPoseHeading = drivetrain.getPoseHeading();
      double newPoseHeading = currentPoseHeading - limelight.getTx();
      if(limelight.isActive()){
        drivetrain.setShootingAngle(newPoseHeading);
      }
      else{
        drivetrain.setShootingAngle(currentPoseHeading);
      }

      // Now get the flywheel up to speed
      if(limelight.isActive()){
        double rpm = Constants.DIST_TO_RPM.get(limelight.getDistance());
        flywheel.runFlywheelSetpoint(rpm + SmartDashboard.getNumber("Teleop: shootLL RPM delta", 0));
      }
      else{
        flywheel.runFlywheelSetpoint(2600 + SmartDashboard.getNumber("Teleop: shootLL RPM delta", 0));          
      }
      
       // Turn hood on for LL shot
      flywheel.setHood(true);
    }

    // Called every time the scheduler runs while the command is scheduled
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true; // End immediately
    }

}
