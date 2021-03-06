package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.utils.Constants;

public class IndexCargo extends CommandBase {
  private Hopper hopper;
  private Intake intake;

  public IndexCargo() {
    hopper = Hopper.getInstance();
    intake = Intake.getInstance();

    addRequirements(hopper);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (!intake.isIntaking()) {
      hopper.stopHopper();
    } else {
      // if there are 0 or 1 balls in the hopper, run the hopper
      if (!hopper.sensesBallTop() || !hopper.sensesBallBottomFiltered()) {
        hopper.runHopper(Constants.HOPPER_INDEX_POWER);
      } else {
        hopper.stopHopper();
      }
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.stopHopper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}