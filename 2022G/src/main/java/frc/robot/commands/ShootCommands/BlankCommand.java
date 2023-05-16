package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BlankCommand extends CommandBase {
  
    /** Creates a new BlankCommand. Does absolutely nothing and ends immediately. */
    public BlankCommand() {

    }

    // Called when the command is initially scheduled
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true; // End immediately
    }

}
