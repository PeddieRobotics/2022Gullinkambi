package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.utils.ControllerMap;
import frc.robot.commands.ClimbCommands.RaiseClimber;
import frc.robot.commands.ClimbCommands.LowerClimber;

public class OI {

  private static OI oi;

  private final Joystick driverXboxController = new Joystick(ControllerMap.DRIVER_PORT);
  private final Joystick operatorXboxController = new Joystick(ControllerMap.OPERATOR_PORT);

  public OI() {
    configureXboxControllers();
  }

  public static OI getInstance() {
    if (oi == null) {
      oi = new OI();
    }

    return oi;
  }

  private void configureXboxControllers() { 
    new JoystickButton(driverXboxController, ControllerMap.XBOX_B).whenPressed(new RaiseClimber());
    new JoystickButton(driverXboxController, ControllerMap.XBOX_A).whenPressed(new LowerClimber());

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
}
  
