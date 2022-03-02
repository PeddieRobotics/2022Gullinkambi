package frc.robot.commands.DriveCommands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants;

/** A command that will turn the robot to the specified angle. */
public class TurnByAngle extends CommandBase{
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */

   private double targetAngleDegrees;
   private double adjustedTargetAngleDegrees;

  public TurnByAngle(double targetAngleDegrees) {
    this.targetAngleDegrees = targetAngleDegrees;
  }

  @Override
  public void initialize(){
    adjustedTargetAngleDegrees = Drivetrain.getInstance().getTurnPID().getSetpoint() + targetAngleDegrees;
  }

  @Override
  public void execute(){
    double output = Drivetrain.getInstance().getTurnPID().calculate(Drivetrain.getInstance().getHeading(), adjustedTargetAngleDegrees);
    if (output > 0) {
      Drivetrain.getInstance().arcadeDrive(0, output + Constants.kTurnToAngleFF);
    } else if (output < 0) {
      Drivetrain.getInstance().arcadeDrive(0, output - Constants.kTurnToAngleFF);
    } else {
      Drivetrain.getInstance().arcadeDrive(0, output);
    }
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return Drivetrain.getInstance().getTurnPID().atSetpoint();
  }
}
