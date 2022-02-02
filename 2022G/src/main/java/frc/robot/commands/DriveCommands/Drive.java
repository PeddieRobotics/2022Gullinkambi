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
        if (Constants.OI_CONFIG == OIConfig.COMPETITION) { // both joystick and xbox, but joysticks are the drivers and xbox is the operator
            //for now this is just the same stuff as joysticks, so it needs to change to something
            double joystickSpeed = oi2.getSpeed();
            double joystickTurn = oi2.getTurn();
            System.out.println("joystickSpeed" + ": " + joystickSpeed);
            System.out.println("joystickTurn)" + ": " + joystickTurn);
            boolean reverse = oi2.getInverseMode(); // inverse and slow only work on joystick
            boolean driveSlow = oi2.getSlowMode();
            
            if (!reverse && !driveSlow) {
                drivetrain.arcadeDrive(joystickSpeed, joystickTurn);
            } else if (!reverse && driveSlow) {
                joystickSpeed = oi2.getSpeed() * Constants.SLOW_SPEED_MULTIPLIER;
                joystickTurn = oi2.getTurn() * Constants.SLOW_TURN_MULTIPLIER;
                drivetrain.arcadeDrive(joystickSpeed, joystickTurn);
            } else if (!driveSlow && reverse) {
                joystickSpeed = -oi2.getSpeed();
                joystickTurn = oi2.getTurn();
                drivetrain.arcadeDrive(joystickSpeed, joystickTurn);
            } else {
                joystickSpeed = -(oi2.getSpeed() * Constants.SLOW_SPEED_MULTIPLIER);
                joystickTurn = oi2.getTurn() * Constants.SLOW_TURN_MULTIPLIER;
                drivetrain.arcadeDrive(joystickSpeed, joystickTurn);
            }
        } else if (Constants.OI_CONFIG == OIConfig.XBOX_TEST) { // xbox
            double speedInput = oi.getSpeed();
            double turnInput = oi.getTurn();
            drivetrain.arcadeDrive(speedInput, turnInput);

        } else if (Constants.OI_CONFIG == OIConfig.JOYSTICK_TEST) { // joystick
            double joystickSpeed = oi2.getSpeed();
            double joystickTurn = oi2.getTurn();
            boolean reverse = oi2.getInverseMode(); // inverse and slow only work on joystick
            boolean driveSlow = oi2.getSlowMode();

            if (!reverse && !driveSlow) {
                drivetrain.arcadeDrive(joystickSpeed, joystickTurn);
            } else if (!reverse && driveSlow) {
                joystickSpeed = oi2.getSpeed() * Constants.SLOW_SPEED_MULTIPLIER;
                joystickTurn = oi2.getTurn() * Constants.SLOW_TURN_MULTIPLIER;
                drivetrain.arcadeDrive(joystickSpeed, joystickTurn);
            } else if (!driveSlow && reverse) {
                joystickSpeed = -oi2.getSpeed();
                joystickTurn = oi2.getTurn();
                drivetrain.arcadeDrive(joystickSpeed, joystickTurn);
            } else {
                joystickSpeed = -(oi2.getSpeed() * Constants.SLOW_SPEED_MULTIPLIER);
                joystickTurn = oi2.getTurn() * Constants.SLOW_TURN_MULTIPLIER;
                drivetrain.arcadeDrive(joystickSpeed, joystickTurn);
            }
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