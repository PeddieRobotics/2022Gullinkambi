package frc.robot.oi;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import frc.robot.commands.IndexCommand;
//import frc.robot.commands.FlywheelCommands.SetFlywheelHood;
import frc.robot.utils.Constants;
import frc.robot.utils.ControllerMap;
import frc.robot.utils.Constants.OIConfig;
import frc.robot.commands.IntakeCommands.IndexCargo;
import frc.robot.commands.IntakeCommands.StartIntake;
import frc.robot.commands.IntakeCommands.StopIntake;
import frc.robot.commands.LightCommands.TurnOn;
import frc.robot.commands.IntakeCommands.UnjamIntake;
import frc.robot.commands.ShootCommands.ShootFar;
import frc.robot.commands.ShootCommands.ShootHigh;
import frc.robot.commands.ShootCommands.ShootLow;

public class XboxOI {

    private static XboxOI oi;
    private Joystick driverXboxController;

    // private final Joystick operatorXboxController = new
    // Joystick(ControllerMap.RIGHT_JOYSTICK_DRIVER_PORT);

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
            // Operator Xbox controller binds
            /*
             * What we want it to do:
             * - increase flywheel speed?
             * - unjam hopper
             * - intake override
             * - hood override
             * - operator buttons to switch the driver from drive mode to climb mode?
             */

            new JoystickButton(driverXboxController,
            ControllerMap.XBOX_X).toggleWhenActive(new UnjamIntake()); // XBOX_X

            //new JoystickButton(driverXboxController, ControllerMap.XBOX_Y).toggleWhenActive(new SetFlywheelHood()); //XBOX_Y

        } else if (Constants.OI_CONFIG == OIConfig.XBOX_TEST) {
            driverXboxController = new Joystick(ControllerMap.XBOX_DRIVER_PORT);
            // Driver xbox controller binds

            // new Button(() -> driverXboxController.getRawAxis(3) > 0.5).whenPressed(new IndexCargo()); // XBOX_RT

            new Button(() -> driverXboxController.getRawAxis(2) > 0.5).whenPressed(new StartIntake()); // XBOX_LT

            new JoystickButton(driverXboxController, ControllerMap.XBOX_LB).whenPressed(new StopIntake()); // XBOX_LB

            new JoystickButton(driverXboxController, ControllerMap.XBOX_RB).whenPressed(new IndexCargo()); // XBOX_RB

            new JoystickButton(driverXboxController, ControllerMap.XBOX_X).whileHeld(new ShootFar()); // XBOX_X

            new JoystickButton(driverXboxController, ControllerMap.XBOX_B).whileHeld(new ShootHigh()); // XBOX_B

            new JoystickButton(driverXboxController, ControllerMap.XBOX_A).whileHeld(new ShootLow()); // XBOX_A

            new JoystickButton(driverXboxController, ControllerMap.XBOX_Y).whileHeld(new UnjamIntake()); // XBOX_Y
        }
    }

    public double getSpeed() {
        return -driverXboxController.getRawAxis(ControllerMap.XBOX_LEFT_STICK_Y);
    }

    public double getTurn() {
        return driverXboxController.getRawAxis(ControllerMap.XBOX_RIGHT_STICK_X);
    }

    public void setControllerRumble(boolean driver, boolean operator) {
        if (driver == true) {
            driverXboxController.setRumble(RumbleType.kLeftRumble, 1);
            driverXboxController.setRumble(RumbleType.kRightRumble, 1);
        } else {
            driverXboxController.setRumble(RumbleType.kLeftRumble, 0);
            driverXboxController.setRumble(RumbleType.kRightRumble, 0);
        }
        if (operator == true) {
            driverXboxController.setRumble(RumbleType.kLeftRumble, 1);
            driverXboxController.setRumble(RumbleType.kRightRumble, 1);
        } else {
            driverXboxController.setRumble(RumbleType.kLeftRumble, 0);
            driverXboxController.setRumble(RumbleType.kRightRumble, 0);
        }
    }

    public boolean getInverseMode() {
        return driverXboxController.getRawButton(6);
    }

}