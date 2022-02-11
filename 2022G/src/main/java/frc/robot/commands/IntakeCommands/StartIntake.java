/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.IntakeCommands;

// import com.team2363.logger.HelixEvents;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.utils.Constants;

public class StartIntake extends CommandBase {

  private Intake intake;

  /** Creates a new StartIntake. */
  public StartIntake() {
    intake = intake.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.runIntake(Constants.INTAKE_SPEED);
    // HelixEvents.getInstance().addEvent("INTAKE", "Initialized: StartIntake");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // HelixEvents.getInstance().addEvent("INTAKE", "Ended: StopIntake");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; // End immediately
  }
}