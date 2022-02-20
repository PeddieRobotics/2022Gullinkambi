
package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ClimbCommands.ExtendArm;
import frc.robot.commands.ClimbCommands.RetractArm;
import frc.robot.commands.IntakeCommands.RunIntake;
import frc.robot.commands.IntakeCommands.StopIntake;
import frc.robot.commands.IntakeCommands.UnjamIntake;
import frc.robot.commands.ShootCommands.ShootWithLL;
import frc.robot.commands.ShootCommands.ShootLayup;
import frc.robot.commands.ShootCommands.ShootLow;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.OIConfig;
import frc.robot.utils.ControllerMap;

public class XboxOI {

    private static XboxOI oi;
    private Joystick driverXboxController;

    public XboxOI() {
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
            driverXboxController = new Joystick(ControllerMap.XBOX_OPERATOR_PORT);

        } else if (Constants.OI_CONFIG == OIConfig.XBOX_TEST) {
            driverXboxController = new Joystick(ControllerMap.XBOX_DRIVER_PORT);
            // Driver xbox controller binds

            new Button(() -> driverXboxController.getRawAxis(3) > 0.5).whenHeld(new ShootLayup()); // XBOX_RT

            new Button(() -> driverXboxController.getRawAxis(2) > 0.5).whenPressed(new RunIntake()); // XBOX_LT

            new JoystickButton(driverXboxController, ControllerMap.XBOX_LB).whenPressed(new StopIntake()); // XBOX_LB

            new JoystickButton(driverXboxController, ControllerMap.XBOX_A).whenHeld(new ShootWithLL()); // XBOX_A

            new JoystickButton(driverXboxController, ControllerMap.XBOX_B).whenHeld(new ShootLow()); // XBOX_B

             new JoystickButton(driverXboxController, ControllerMap.XBOX_Y).whenHeld(new UnjamIntake()).whenReleased(new RunIntake()); // XBOX_Y

             new JoystickButton(driverXboxController, ControllerMap.XBOX_X).whenPressed(new ExtendArm()); // XBOX_X

             new JoystickButton(driverXboxController, ControllerMap.XBOX_START).whenPressed(new RetractArm()); // XBOX_START


        }
    }

    public double getSpeed() {
        return -driverXboxController.getRawAxis(ControllerMap.XBOX_LEFT_STICK_Y);
    }

    public double getTurn() {
        return driverXboxController.getRawAxis(ControllerMap.XBOX_RIGHT_STICK_X);
    }

    public boolean getInverseMode() {
        return driverXboxController.getRawButton(ControllerMap.XBOX_RB);
    }

}