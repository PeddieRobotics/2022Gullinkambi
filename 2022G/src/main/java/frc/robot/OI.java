package frc.robot;

import java.sql.Driver;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.oi.JoystickOI;
import frc.robot.oi.XboxOI;
// import frc.robot.commands.SplitFFRamseteCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.OIConfig;

public class OI {

  private final Drivetrain m_driveTrain;
  private static OI oi;
  private XboxOI m_XboxOI;
  private JoystickOI m_JoystickOI;

  private Joystick leftJoystick, rightJoystick;
  public OI() {

    m_driveTrain = Drivetrain.getInstance();

    if (Constants.OI_CONFIG == OIConfig.COMPETITION) {
      m_XboxOI = XboxOI.getInstance();
      m_XboxOI.configureXboxControllers();
      m_JoystickOI = JoystickOI.getInstance();
      m_JoystickOI.initializeJoysticks();
      m_JoystickOI.configureJoysticks();
      // m_driveTrain.setJoysticks(leftJoystick, rightJoystick);
    } else if (Constants.OI_CONFIG == OIConfig.XBOX_TEST) {
      m_XboxOI = XboxOI.getInstance();
      m_XboxOI.configureXboxControllers();
    } else if (Constants.OI_CONFIG == OIConfig.JOYSTICK_TEST) {
      m_JoystickOI = JoystickOI.getInstance();
      m_JoystickOI.initializeJoysticks();
      m_JoystickOI.configureJoysticks();
      // m_driveTrain.setJoysticks(leftJoystick, rightJoystick);
    }

  }

  public static OI getInstance() {

    if (oi == null) {
      oi = new OI();
    }
    return oi;
  }
}