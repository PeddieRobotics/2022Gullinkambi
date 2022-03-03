package frc.robot.commands.DriveCommands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

   private double targetAngleDegrees;

  public TurnToAngle(double targetAngleDegrees) {
    this.targetAngleDegrees = targetAngleDegrees;
  }

  @Override
  public void initialize(){
    Drivetrain.getInstance().getTurnPID().setSetpoint(targetAngleDegrees);
  }

  @Override
  public void execute(){
    double output = Drivetrain.getInstance().getTurnPID().calculate(Drivetrain.getInstance().getPoseHeading(), targetAngleDegrees);
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
    return (Math.abs(Drivetrain.getInstance().getPoseHeading()-targetAngleDegrees) < Constants.kTurnByAngleToleranceDeg);
    // End when the controller is at the reference
    //SmartDashboard.putNumber("turn pid setpoint", Drivetrain.getInstance().getTurnPID().getPositionError());
    //return Drivetrain.getInstance().getTurnPID().atSetpoint();
  }
}
