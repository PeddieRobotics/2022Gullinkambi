
package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ClimbCommands.ExtendRightArm;
import frc.robot.commands.ClimbCommands.InitializeRightArm;
import frc.robot.commands.ClimbCommands.RetractRightArm;
import frc.robot.commands.IntakeCommands.RunIntake;
import frc.robot.commands.IntakeCommands.RunIntakeEndImmediately;
import frc.robot.commands.IntakeCommands.StopIntake;
import frc.robot.commands.IntakeCommands.UnjamIntake;
import frc.robot.commands.ShootCommands.ShootWithLL;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.commands.ShootCommands.ShootLayup;
import frc.robot.commands.ShootCommands.ShootLow;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.OIConfig;
import frc.robot.utils.ControllerMap;

public class XboxOI {

    private static XboxOI oi;
    private Joystick xboxController;
    private Drivetrain drivetrain;
    private Intake intake;

    public XboxOI() {
        drivetrain = Drivetrain.getInstance();
        intake = Intake.getInstance();

        configureXboxControllers();
    }

    public static XboxOI getInstance() {
        if (oi == null) {
            oi = new XboxOI();
        }

        return oi;
    }

    public void configureXboxControllers() {
        if (Constants.OI_CONFIG == OIConfig.COMPETITION) {

            xboxController = new Joystick(ControllerMap.XBOX_OPERATOR_PORT);

            new JoystickButton(xboxController, ControllerMap.XBOX_Y).whenHeld(new ShootLow()); // XBOX_Y
            new JoystickButton(xboxController, ControllerMap.XBOX_X).whenPressed(new UnjamIntake(true).andThen(new WaitCommand(0.25)).andThen(new RunIntakeEndImmediately())); // XBOX_X
            new JoystickButton(xboxController, ControllerMap.XBOX_A).whenHeld(new UnjamIntake(false)).whenReleased(new RunIntake()); // XBOX_A
            new JoystickButton(xboxController, ControllerMap.XBOX_B).whenPressed(new InitializeRightArm()); // XBOX_B
            //new JoystickButton(xboxController, ControllerMap.XBOX_RB).toggleWhenPressed(new ConditionalCommand(new InstantCommand(drivetrain::setToCoastMode, drivetrain), new InstantCommand(drivetrain::setToBrakeMode, drivetrain), drivetrain::isBrakeMode)); //XBOX_RB

        } else if (Constants.OI_CONFIG == OIConfig.XBOX_TEST) {
            xboxController = new Joystick(ControllerMap.XBOX_DRIVER_PORT);
            // Driver xbox controller binds    
            new Button(() -> xboxController.getRawAxis(3) > 0.5).whenHeld(new ShootLayup(false)); // XBOX_RT

            new Button(() -> xboxController.getRawAxis(2) > 0.5).toggleWhenPressed(new ConditionalCommand(new StopIntake(), new RunIntake(), intake::getIntakeSolenoid)); // XBOX_LT

            new JoystickButton(xboxController, ControllerMap.XBOX_LB).toggleWhenPressed(new ConditionalCommand(new InstantCommand(drivetrain::setToRegularMode, drivetrain), new InstantCommand(drivetrain::setToInverseMode, drivetrain), drivetrain::isInverseMode));
             // XBOX_LB

            new JoystickButton(xboxController, ControllerMap.XBOX_A).whenHeld(new ShootWithLL(false)); // XBOX_A

            new JoystickButton(xboxController, ControllerMap.XBOX_X).whenHeld(new ExtendRightArm()).whenReleased(new RetractRightArm()); // XBOX_X

        }
    }

    public double getSpeed() {
        return -xboxController.getRawAxis(ControllerMap.XBOX_LEFT_STICK_Y);
    }

    public double getTurn() {
        return xboxController.getRawAxis(ControllerMap.XBOX_RIGHT_STICK_X);
    }

}