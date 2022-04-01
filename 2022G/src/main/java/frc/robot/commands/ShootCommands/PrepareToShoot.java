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
      if(limelight.hasTarget()){
        // Set to brake mode
        drivetrain.setBrake();
        // Turn hood on for LL shot
        // flywheel.setHood(true);

        double currentPoseHeading = drivetrain.getPoseHeading();

        // Check if the limelight isn't working, if so we want to end immediately and not try to target
        if(limelight.isActive()){
          // Get horizontal angular error from limelight and calculate new heading
          double d = limelight.getDistance();
          double tx = Math.toRadians(limelight.getTx());
          double offset = 5.0; // inches from limelight to gyro
          double h = Constants.TARGET_HEIGHT - Constants.LL_HEIGHT;
          double theta1 = Math.acos((-h*h + (d*d + h*h)*Math.cos(tx))/(d*d));
          double theta2 = Math.toDegrees(Math.atan(d*Math.sin(theta1)/(d*Math.cos(theta1)+offset)));
          SmartDashboard.putNumber("d", d);
          SmartDashboard.putNumber("h", h);
          SmartDashboard.putNumber("theta1", Math.toDegrees(theta1));
          SmartDashboard.putNumber("theta2", theta2);
          double newPoseHeading = currentPoseHeading - Math.signum(tx)*theta2;
  
          double distNew = d * Math.sin(theta1)/Math.sin(Math.toRadians(theta2));
          SmartDashboard.putNumber("New dist", distNew);

          // Now get the flywheel up to speed
          drivetrain.setShootingAngle(newPoseHeading);
          // double rpm = Constants.DIST_TO_RPM.get(d);
          // flywheel.runFlywheelSetpoint(rpm + SmartDashboard.getNumber("Teleop: shootLL RPM delta", 0));

        }
        else{
          limelightBroken = true;
          drivetrain.setShootingAngle(currentPoseHeading);
          flywheel.runFlywheelSetpoint(2600 + SmartDashboard.getNumber("Teleop: shootLL RPM delta", 0));          
        }
        
      }
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
