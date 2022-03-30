package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ClimbCommands.ExtendArm;
import frc.robot.commands.ClimbCommands.RetractArm;
import frc.robot.commands.DriveCommands.TargetToAngle;
import frc.robot.commands.DriveCommands.TurnToAngle;
import frc.robot.commands.IntakeCommands.RunIntake;
import frc.robot.commands.IntakeCommands.StopIntake;
import frc.robot.commands.ShootCommands.BlankCommand;
import frc.robot.commands.ShootCommands.PrepareToShoot;
import frc.robot.commands.ShootCommands.ShootLayup;
import frc.robot.commands.ShootCommands.ShootWithLL;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.ControllerMap;

public class JoystickOI {

  private static JoystickOI oi;

  private Joystick leftJoystick, rightJoystick;

  private Drivetrain drivetrain;
  private Intake intake;
  private Limelight limelight;

  private JoystickButton leftTrigger, leftButton2, leftButton3, leftButton4;
  private JoystickButton rightTrigger, rightButton2, rightButton3, rightButton4;

  public JoystickOI() {
    drivetrain = Drivetrain.getInstance();
    intake = Intake.getInstance();
    limelight = Limelight.getInstance();
    
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
    
    rightTrigger.whenHeld(new ShootLayup(false));
    rightButton2.whenHeld(new ConditionalCommand(new SequentialCommandGroup(new PrepareToShoot(), new TargetToAngle(), new ShootWithLL(false)), new BlankCommand(), limelight::hasTarget));
    rightButton3.whenHeld(new ExtendArm()).whenReleased(new RetractArm());
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