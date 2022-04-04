package frc.robot.commands.DriveCommands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants;

/** A command that will turn the robot to the specified angle. */
public class TurnToAngle extends CommandBase{
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */

  private Drivetrain drivetrain;

   private double targetAngleDegrees;

  public TurnToAngle(double targetAngleDegrees) {
    this.targetAngleDegrees = targetAngleDegrees;

    drivetrain = Drivetrain.getInstance();

    addRequirements(drivetrain);
  }

  @Override
  public void initialize(){
    drivetrain.getTurnPID().setSetpoint(targetAngleDegrees);
  }

  @Override
  public void execute(){
    double output = drivetrain.getTurnPID().calculate(drivetrain.getPoseHeading(), targetAngleDegrees);
    if (output > 0) {
      drivetrain.arcadeDrive(0, output + Constants.kTurnToAngleFF);
    } else if (output < 0) {
      drivetrain.arcadeDrive(0, output - Constants.kTurnToAngleFF);
    } else {
      drivetrain.arcadeDrive(0, output);
    }
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(drivetrain.getPoseHeading()-targetAngleDegrees) < Constants.kTurnToAngleToleranceDeg);
  }
}
