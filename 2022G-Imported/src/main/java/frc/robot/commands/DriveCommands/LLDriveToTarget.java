package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class LLDriveToTarget extends CommandBase {

    // These numbers must be tuned for your Robot!  Be careful!
    final double STEER_K = 0.03;                    // how hard to turn toward the target
    final double DRIVE_K = 0.02;                    // how hard to drive fwd toward the target
    final double DESIRED_DISTANCE = 80.0;        // Distance for safe limelight shot
    final double MAX_SPEED = 0.5;                   // Simple speed limit so we don't drive too fast
    final double MAX_TURN = 0.5;                   // Simple turn rate limit so we don't spin too fast

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
            // don't let the robot spin too fast
            if (Math.abs(turn_cmd) > MAX_TURN){
                turn_cmd = Math.signum(limelight.getTx())*MAX_TURN;
            }
            limelightTurn = turn_cmd;

            // try to drive forward until the target area reaches our desired area
            double speed_cmd = (DESIRED_DISTANCE - limelight.getDistance()) * DRIVE_K;

            // don't let the robot drive too fast into the goal
            if (speed_cmd > MAX_SPEED)
            {
            speed_cmd = Math.signum(limelight.getTx())*MAX_SPEED;
            }
            limelightSpeed = speed_cmd;
        }
        drivetrain.arcadeDrive(-limelightSpeed, -limelightTurn);
        SmartDashboard.putNumber("LL speed", -limelightSpeed);
        SmartDashboard.putNumber("LL turn", -limelightTurn);

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
