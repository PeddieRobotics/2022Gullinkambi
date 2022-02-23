package frc.robot.utils.LoggingCustoms;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.*;


/**
 * Manages logging general system data. This is NOT replayed to the simulator.
 */
public class LoggedSystemStatsCustom implements LoggableInputs {

  private static LoggedSystemStatsCustom instance;
  private static final PowerDistribution pdp = new PowerDistribution(20, ModuleType.kRev);

  private LoggedSystemStatsCustom() {
  }

  public static LoggedSystemStatsCustom getInstance() {
    if (instance == null) {
      instance = new LoggedSystemStatsCustom();
    }
    return instance;
  }

  public void toLog(LogTable table) {
    table.put("BatteryVoltage", RobotController.getBatteryVoltage());
    table.put("BrownedOut", RobotController.isBrownedOut());
    table.put("CANBusUtilization", RobotController.getCANStatus().percentBusUtilization);
    int channelCount = pdp.getNumChannels();
    double[] pdpCurrents = new double[channelCount];
    for (int channel = 0; channel < channelCount; channel++) {
      pdpCurrents[channel] = pdp.getCurrent(channel);
    }
    table.put("PDPCurrents", pdpCurrents);
  }

  public void fromLog(LogTable table) {
    // Ignore replayed inputs
  }
}
