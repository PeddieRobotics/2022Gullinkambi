
package frc.robot.oi;

import frc.robot.utils.Constants;
import frc.robot.utils.Constants.OIConfig;

public class OI {

  private static OI oi;
  private XboxOI m_XboxOI;
  private JoystickOI m_JoystickOI;
  private PS5OI m_PS5OI;

  public OI() {

    if (Constants.OI_CONFIG == OIConfig.COMPETITION) {

      m_XboxOI = XboxOI.getInstance();
      m_JoystickOI = JoystickOI.getInstance();

    } else if (Constants.OI_CONFIG == OIConfig.XBOX_TEST) {

      m_XboxOI = XboxOI.getInstance();

    } else if (Constants.OI_CONFIG == OIConfig.JOYSTICK_TEST) {

      m_JoystickOI = JoystickOI.getInstance();
    }
    else if (Constants.OI_CONFIG == OIConfig.PS5_TEST){
      m_PS5OI = PS5OI.getInstance();
    }

  }

  public static OI getInstance() {

    if (oi == null) {
      oi = new OI();
    }
    return oi;
  }
}
