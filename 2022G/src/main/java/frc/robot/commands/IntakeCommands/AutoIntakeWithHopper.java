
package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.utils.Constants;

public class AutoIntakeWithHopper extends CommandBase {

  private Intake intake;
  private Hopper hopper;

  /** Creates a new AutoIntakeWithHopper. */
  public AutoIntakeWithHopper() {
    intake = Intake.getInstance();
    hopper = Hopper.getInstance();

    addRequirements(intake, hopper);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setIntakeSolenoid(true);
    intake.setIntakeSpeed(SmartDashboard.getNumber("Teleop: Intake speed", Constants.INTAKE_SPEED));
    hopper.runHopper(SmartDashboard.getNumber("Teleop: Hopper speed", Constants.HOPPER_INDEX_SPEED));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}