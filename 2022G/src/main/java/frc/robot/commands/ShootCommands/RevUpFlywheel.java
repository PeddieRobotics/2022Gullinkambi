package frc.robot.commands.ShootCommands;
import frc.robot.subsystems.Intake;
// import com.team2363.logger.HelixEvents;
import frc.robot.utils.Constants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RevUpFlywheel extends CommandBase {
    
    private Intake intake;
    private Flywheel flywheel;
    private Hopper hopper;

    /** Creates a new RevUpFlywheel. */
    public RevUpFlywheel() {   
        intake = Intake.getInstance();
        flywheel = Flywheel.getInstance();
        hopper = Hopper.getInstance();

        addRequirements(flywheel, intake);

    }

    //Called when the command is initially scheduled
    @Override
    public void initialize() {}

    //Called every time the scheduler runs while the command is scheduled
    @Override
    public void execute() {
        if (hopper.sensesBallBottom() && hopper.sensesBallTop()) {
            intake.stopIntake();
            flywheel.runFlywheelSetpoint(Constants.FLYWHEEL_RPM_REV_UP);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
