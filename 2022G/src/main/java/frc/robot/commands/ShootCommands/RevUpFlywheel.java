package frc.robot.commands.ShootCommands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.Constants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RevUpFlywheel extends CommandBase {

    private Intake intake;
    private Flywheel flywheel;
    private Hopper hopper;
    private Limelight limelight;

    /** Creates a new RevUpFlywheel. */
    public RevUpFlywheel() {
        intake = Intake.getInstance();
        flywheel = Flywheel.getInstance();
        hopper = Hopper.getInstance();
        limelight = Limelight.getInstance();

        addRequirements(flywheel, intake, limelight);

    }

    // Called when the command is initially scheduled
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled
    @Override
    public void execute() {
        if (SmartDashboard.getBoolean("RevUp AI", true)) {
            if (hopper.sensesBallBottomFiltered() && hopper.sensesBallTop()) {
                intake.stopIntake();
                if(limelight.hasTarget() && Math.abs(limelight.getTxAverage()) < 10){
                    double dist = limelight.getDistance();
                    if(dist > 110 && dist < 140){
                        flywheel.runFlywheelSetpoint(Constants.FLYWHEEL_RPM_REV_UP_MEDIUM);
                    }
                    else if(dist >= 140){
                        flywheel.runFlywheelSetpoint(Constants.FLYWHEEL_RPM_REV_UP_FAR);
                    }
                }
                else{
                    flywheel.runFlywheelSetpoint(Constants.FLYWHEEL_RPM_REV_UP_STANDARD);
                }
            }
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
