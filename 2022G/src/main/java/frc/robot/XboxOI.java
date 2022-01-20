package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.utils.ControllerMap;
import frc.robot.commands.ClimbCommands.RaiseClimber;
import frc.robot.commands.ClimbCommands.LowerClimber;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Turret.TurretMode;

public class XboxOI {

  private static XboxOI oi;

  private final Joystick driverXboxController = new Joystick(ControllerMap.DRIVER_PORT);
  private final Joystick operatorXboxController = new Joystick(ControllerMap.OPERATOR_PORT);
  public final JoystickButton override = new JoystickButton(operatorXboxController, ControllerMap.XBOX_A);

  public XboxOI() {
    configureXboxControllers();
  }

  public static XboxOI getInstance() {
    if (oi == null) {
      oi = new XboxOI();
    }

    return oi;
  }

  private void configureXboxControllers() { 
    new JoystickButton(driverXboxController, ControllerMap.XBOX_B).whenPressed(new RaiseClimber());
    new JoystickButton(driverXboxController, ControllerMap.XBOX_A).whenPressed(new LowerClimber());
    override.toggleWhenPressed(new StartEndCommand(Turret.getInstance()::setOverride, Turret.getInstance()::setPreviousMode,
        Turret.getInstance()));
  }

  public double getSpeed() {
    return -driverXboxController.getRawAxis(ControllerMap.XBOX_LEFT_STICK_Y);
  }

  public double getTurn() {
    return driverXboxController.getRawAxis(ControllerMap.XBOX_RIGHT_STICK_X);
  }

  public double getTurretInputFromOperatorThumbstick(){
    return operatorXboxController.getRawAxis(ControllerMap.XBOX_LEFT_STICK_X);
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
  
