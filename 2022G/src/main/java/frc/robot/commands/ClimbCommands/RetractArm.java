package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Climber;

/** An example command that uses an example subsystem. */
public class RetractArm extends CommandBase {
  private Climber climber;
  boolean nearLimitSensor;

  public RetractArm() {
    climber = Climber.getInstance();
    addRequirements(climber);

    nearLimitSensor = false;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.moveToPosition(0); // Hold at encoder position 0
    //climber.disablePIDController();
    //climber.run(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(climber.getEncoderPosition() > -1 && !nearLimitSensor){
      nearLimitSensor = true;
      climber.disablePIDController();
      climber.run(0.5);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.run(0);
    climber.enablePIDController();
    climber.moveToPosition(0); // Hold at encoder position 0
  }

  // Returns true when the command should end.
  // runs everytime execute runs
  @Override
  public boolean isFinished() {
    return climber.armSensorState();
  }
}
