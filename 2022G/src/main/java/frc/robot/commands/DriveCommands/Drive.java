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
    private XboxOI oi;
    private JoystickOI oi2;

    public Drive() {
        drivetrain = Drivetrain.getInstance();
        if (Constants.OI_CONFIG == OIConfig.COMPETITION) {
            oi = XboxOI.getInstance();
            oi2 = JoystickOI.getInstance();
        } else if (Constants.OI_CONFIG == OIConfig.XBOX_TEST) {
            oi = XboxOI.getInstance();
        } else if (Constants.OI_CONFIG == OIConfig.JOYSTICK_TEST) {
            oi2 = JoystickOI.getInstance();
        }

        addRequirements(drivetrain);
    }

    // Initializes the drive command
    @Override
    public void initialize() {
    }

    // Executes the drive command
    @Override
    public void execute() {

        boolean reverse = false;
        double speedInput = 0.0;
        double turnInput = 0.0;

        if (Constants.OI_CONFIG == OIConfig.COMPETITION) { // both joystick and xbox, but joysticks are the drivers and
                                                           // xbox is the operator
            // for now this is just the same stuff as joysticks, so it needs to change to
            // something
            speedInput = oi2.getSpeed();
            turnInput = oi2.getTurn();
            reverse = oi2.getInverseMode();

        } else if (Constants.OI_CONFIG == OIConfig.XBOX_TEST) { // xbox

            speedInput = oi.getSpeed();
            turnInput = oi.getTurn();
            reverse = oi.getInverseMode();

        } else if (Constants.OI_CONFIG == OIConfig.JOYSTICK_TEST) { // joystick

            speedInput = oi2.getSpeed();
            turnInput = oi2.getTurn();
            reverse = oi2.getInverseMode();

        }
        // if (!reverse) {
        // drivetrain.arcadeDrive(speedInput, turnInput);
        // } else {
        // drivetrain.arcadeDrive(-speedInput, turnInput);
        // }
        if (!reverse) {
            drivetrain.curvatureDrive(speedInput, turnInput);
        } else {
            drivetrain.curvatureDrive(-speedInput, turnInput);
        }

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