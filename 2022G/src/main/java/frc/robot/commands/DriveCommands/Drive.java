package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.oi.JoystickOI;
import frc.robot.oi.XboxOI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.OIConfig;

public class Drive extends CommandBase {

    private Drivetrain drivetrain;
    private XboxOI xboxOI;
    private JoystickOI joystickOI;

    public Drive() {
        drivetrain = Drivetrain.getInstance();

        if (Constants.OI_CONFIG == OIConfig.COMPETITION) {
            xboxOI = XboxOI.getInstance();
            joystickOI = JoystickOI.getInstance();
        } else if (Constants.OI_CONFIG == OIConfig.XBOX_TEST) {
            xboxOI = XboxOI.getInstance();
        } else if (Constants.OI_CONFIG == OIConfig.JOYSTICK_TEST) {
            joystickOI = JoystickOI.getInstance();
        }

        addRequirements(drivetrain);
    }

    // Initializes the drive command
    @Override
    public void initialize() {
        // If we're driving, we're not locked on a limelight target. Shouldn't be needed, but placed here as a safety on the logic elsewhere.
        drivetrain.setLockedOnTarget(false);
    }

    // Executes the drive command
    @Override
    public void execute() {
        // Local variables for this particular execute loop. May be modified by checking elsewhere. (Inefficient/consider changing implementation)
        boolean reverse = false;
        boolean brakeMode = false;
        double speedInput = 0.0;
        double turnInput = 0.0;

        if (Constants.OI_CONFIG == OIConfig.COMPETITION) { // both joystick and xbox, but joysticks are the drivers and
            // xbox is the operator
            // for now this is just the same stuff as joysticks, so it needs to change to
            // something
            speedInput = joystickOI.getSpeed();
            turnInput = -joystickOI.getTurn();
            reverse = drivetrain.isInverseMode();
            brakeMode = drivetrain.isBrakeMode();

        } else if (Constants.OI_CONFIG == OIConfig.XBOX_TEST) { // xbox

            speedInput = xboxOI.getSpeed();
            turnInput = -xboxOI.getTurn();
            reverse = drivetrain.isInverseMode();

        } else if (Constants.OI_CONFIG == OIConfig.JOYSTICK_TEST) { // joystick

            speedInput = joystickOI.getSpeed();
            turnInput = -joystickOI.getTurn();
            reverse = drivetrain.isInverseMode();

        }

        if (!reverse) {
            drivetrain.curvatureDrive(speedInput, turnInput);
        } else {
            drivetrain.curvatureDrive(-speedInput, turnInput);
        }

        // if(!brakeMode){
        //     drivetrain.setCoast();
        // } else {
        //     drivetrain.setBrake();
        // }

    }

    // End the command if it is interrupted
    @Override
    public void end(boolean interrupted) {
    }

    // Checks if the command is finished
    @Override
    public boolean isFinished() {
        return false;
    }
}