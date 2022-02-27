package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class LLDriveToTarget extends CommandBase {

    // These numbers must be tuned for your Robot!  Be careful!
    final double STEER_K = 0.03;                    // how hard to turn toward the target
    final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
    final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
    final double MAX_DRIVE = 0.5;                   // Simple speed limit so we don't drive too fast

    private Limelight limelight;
    private Drivetrain drivetrain;

    private double limelightSpeed;
    private double limelightTurn;

    private double defaultSpeed;
    private double defaultTurn;

    public LLDriveToTarget(double defaultSpeed, double defaultTurn){
        limelight = Limelight.getInstance();
        drivetrain = Drivetrain.getInstance();

        limelightSpeed = 0.0;
        limelightTurn = 0.0;

        this.defaultSpeed = defaultSpeed;
        this.defaultTurn = defaultTurn;
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        if (limelight.hasTarget())
        {
          limelightSpeed = defaultTurn;
          limelightTurn = defaultSpeed;
        }
        else{
            // Start with proportional steering
            double turn_cmd = limelight.getTx() * STEER_K;
            limelightTurn = turn_cmd;

            // try to drive forward until the target area reaches our desired area
            double speed_cmd = (DESIRED_TARGET_AREA - limelight.getTa()) * DRIVE_K;

            // don't let the robot drive too fast into the goal
            if (speed_cmd > MAX_DRIVE)
            {
            speed_cmd = MAX_DRIVE;
            }
            limelightSpeed = speed_cmd;
        }
        drivetrain.arcadeDrive(limelightSpeed, limelightTurn);

    }

    @Override
    public void end(boolean interrupted){
        drivetrain.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
