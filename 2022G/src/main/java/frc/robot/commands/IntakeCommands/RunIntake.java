/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.utils.Constants;

public class RunIntake extends CommandBase {

  private Intake intake;
  private Hopper hopper;
  private Flywheel flywheel;

  /** Creates a new StartIntake. */
  public RunIntake() {
    intake = Intake.getInstance();
    hopper = Hopper.getInstance();
    flywheel = Flywheel.getInstance();
    addRequirements(intake, flywheel);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setIntakeSolenoid(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntakeSpeed(SmartDashboard.getNumber("Teleop: Intake speed", Constants.INTAKE_SPEED));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    // Rev up the flywheel ONLY if this command is ending from a double sensor trigger
    if((hopper.sensesBallBottom() && hopper.sensesBallTop())){
      flywheel.runFlywheelSetpoint(Constants.FLYWHEEL_RPM_REV_UP);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (hopper.sensesBallBottom() && hopper.sensesBallTop()); 
  }
}