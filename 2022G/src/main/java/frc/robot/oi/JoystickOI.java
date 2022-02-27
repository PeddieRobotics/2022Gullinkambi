package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ClimbCommands.ExtendArm;
import frc.robot.commands.ClimbCommands.RetractArm;
import frc.robot.commands.DriveCommands.LLDriveToTarget;
import frc.robot.commands.DriveCommands.TurnByAngle;
import frc.robot.commands.IntakeCommands.RunIntake;
import frc.robot.commands.IntakeCommands.StopIntake;
import frc.robot.commands.ShootCommands.ShootLayup;
import frc.robot.commands.ShootCommands.ShootWithLL;
import frc.robot.commands.ShootCommands.Target;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.utils.ControllerMap;

public class JoystickOI {

  private static JoystickOI oi;

  private Joystick leftJoystick, rightJoystick;

  private Drivetrain drivetrain;
  private Intake intake;

  private JoystickButton leftTrigger, leftButton2, leftButton3, leftButton4;
  private JoystickButton rightTrigger, rightButton2, rightButton3, rightButton4;
  private JoystickButton opTrigger, opButton2, opButton3, opButton4, opButton5, opButton6, opButton7, opButton8, opButton9, opButton10, opButton11, opButton12;
  private JoystickButton driverButtonA, driverButtonB, driverButtonX, driverButtonY, driverButtonLeftBumper,
      driverButtonRightBumper, driverButtonBack, driverButtonStart, driverButtonLeftStick, driverButtonRightStick;

  public JoystickOI() {
    drivetrain = Drivetrain.getInstance();
    intake = Intake.getInstance();
    
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
    leftTrigger.toggleWhenPressed(new ConditionalCommand(new StopIntake(), new RunIntake(), intake::getIntakeSolenoid));
    leftButton2.toggleWhenPressed(new ConditionalCommand(new InstantCommand(drivetrain::setToRegularMode, drivetrain), new InstantCommand(drivetrain::setToInverseMode, drivetrain), drivetrain::isInverseMode));
    
    rightTrigger.whenHeld(new ShootLayup());
    rightButton2.whenHeld(new ParallelCommandGroup(new Target(), new ShootWithLL()));
    //rightButton3.whenHeld(new ExtendArm()).whenReleased(new RetractArm());
    //rightButton4.whenHeld(new Target());
    rightButton4.whenPressed(new TurnByAngle(90));
    rightButton3.whenHeld(new LLDriveToTarget(0.35,0));
  }

  public double getSpeed() {
    return -leftJoystick.getRawAxis(1);
  }

  public double getTurn() {
    return rightJoystick.getRawAxis(0);
  }

  public static JoystickOI getInstance() {
    if (oi == null) {
      oi = new JoystickOI();
    }
    return oi;
  }
}