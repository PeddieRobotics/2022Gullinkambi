
package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

public class AutoIntakeWithHopper extends CommandBase {

  private Intake intake;
  private Hopper hopper;
  private double indexSpeed, hopperSpeed;

  /** Creates a new AutoIntakeWithHopper. */
  public AutoIntakeWithHopper(double intakeSpeed, double hopperSpeed) {
    intake = Intake.getInstance();
    hopper = Hopper.getInstance();
    this.indexSpeed = intakeSpeed;
    this.hopperSpeed = hopperSpeed;
    addRequirements(intake, hopper);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setIntakeSolenoid(true);
    intake.setIntakeSpeed(indexSpeed);
    hopper.runHopper(hopperSpeed);
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