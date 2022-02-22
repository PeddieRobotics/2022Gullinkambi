package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeCommands.RunIntake;
import frc.robot.commands.IntakeCommands.StopIntake;
import frc.robot.commands.IntakeCommands.UnjamIntake;
import frc.robot.commands.ShootCommands.ShootWithLL;
import frc.robot.commands.ShootCommands.ShootLayup;
import frc.robot.commands.ShootCommands.ShootLow;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.utils.ControllerMap;

public class JoystickOI {

  private static JoystickOI oi;

  private Joystick leftJoystick, rightJoystick;

  private JoystickButton leftTrigger, leftButton2, leftButton3, leftButton4;
  private JoystickButton rightTrigger, rightButton2, rightButton3, rightButton4;
  private JoystickButton opTrigger, opButton2, opButton3, opButton4, opButton5, opButton6, opButton7, opButton8, opButton9, opButton10, opButton11, opButton12;
  private JoystickButton driverButtonA, driverButtonB, driverButtonX, driverButtonY, driverButtonLeftBumper,
      driverButtonRightBumper, driverButtonBack, driverButtonStart, driverButtonLeftStick, driverButtonRightStick;

  // private XboxTrigger driverLeftTrigger, driverRightTrigger;

  public JoystickOI() {

    initializeJoysticks();
    configureJoysticks();

  }

  public void initializeJoysticks() {

    leftJoystick = new Joystick(ControllerMap.LEFT_JOYSTICK_DRIVER_PORT);
    rightJoystick = new Joystick(ControllerMap.RIGHT_JOYSTICK_DRIVER_PORT);

    leftTrigger = new JoystickButton(leftJoystick, 1);
    leftButton2 = new JoystickButton(leftJoystick, 2);
    leftButton3 = new JoystickButton(leftJoystick, 3);
    leftButton4 = new JoystickButton(leftJoystick, 4);

    rightTrigger = new JoystickButton(rightJoystick, 1);
    rightButton2 = new JoystickButton(rightJoystick, 2);
    rightButton3 = new JoystickButton(rightJoystick, 3);
    rightButton4 = new JoystickButton(rightJoystick, 4);

  }

  public void configureJoysticks() {

    // Driver joystick binds (dual joystick)
    leftButton2.whenHeld(new UnjamIntake()).whenReleased(new RunIntake());
    leftButton3.whenPressed(new RunIntake());
    leftButton4.whenPressed(new StopIntake());
    
    rightButton2.whenHeld(new ShootLayup());
    rightButton3.whenHeld(new ShootWithLL());
    rightButton4.whenHeld(new ShootLow());

  }

  public double getSpeed() {
    return -leftJoystick.getRawAxis(1);
  }

  public double getTurn() {
    return rightJoystick.getRawAxis(0);
  }

  
  public boolean getInverseMode() {
    // checks if left trigger is pressed (left trigger id is 1)
    SmartDashboard.putBoolean("isLeftTriggerPressed", leftJoystick.getRawButton(1));
    return leftJoystick.getRawButton(1);
  }

  public boolean getSlowMode() {
    // checks if right trigger is pressed (right trigger id is 1)
    SmartDashboard.putBoolean("isRightTriggerPressed", rightJoystick.getRawButton(1));
    return rightJoystick.getRawButton(1);
  }

  public static JoystickOI getInstance() {
    if (oi == null) {
      oi = new JoystickOI();
    }
    return oi;
  }
}