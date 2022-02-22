package frc.robot.utils.LoggingCustoms;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.IterativeRobotBase;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.NotifierJNI;

/**
 * TimedRobot implements the IterativeRobotBase robot program framework.
 *
 * <p>
 * The TimedRobot class is intended to be subclassed by a user creating a robot
 * program.
 *
 * <p>
 * periodic() functions from the base class are called on an interval by a
 * Notifier instance.
 */
public class LoggedRobotCustom extends IterativeRobotBase {

  public static final double defaultPeriod = 0.02;

  private final int notifier = NotifierJNI.initializeNotifier();

  private final double period;

  private double nextCycle;
  private boolean useTiming = true;

  /** Constructor for LoggedRobot. */
  protected LoggedRobotCustom() {
    this(defaultPeriod);
  }

  /**
   * Constructor for TimedRobot.
   *
   * @param period Period in seconds.
   */
  protected LoggedRobotCustom(double period) {
    super(period);
    this.period = period;
    NotifierJNI.setNotifierName(notifier, "LoggedRobot");

    HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_Timed);
  }

  @Override
  @SuppressWarnings("NoFinalizer")
  protected void finalize() {
    NotifierJNI.stopNotifier(notifier);
    NotifierJNI.cleanNotifier(notifier);
  }

  /** Provide an alternate "main loop" via startCompetition(). */
  @Override
  @SuppressWarnings("UnsafeFinalization")
  public void startCompetition() {
    robotInit();

    if (isSimulation()) {
      simulationInit();
    }

    // Save data from init cycle
    LoggerCustom.getInstance().periodicAfterUser();

    // Tell the DS that the robot is ready to be enabled
    HAL.observeUserProgramStarting();

    // Loop forever, calling the appropriate mode-dependent function
    nextCycle = LoggerCustom.getInstance().getRealTimestamp();
    while (true) {
      if (useTiming) {
        NotifierJNI.updateNotifierAlarm(notifier, (long) (nextCycle * 1e6));
        long curTime = NotifierJNI.waitForNotifierAlarm(notifier);
        if (curTime == 0) {
          break;
        }
        nextCycle += period;
      }

      double loopCycleStart = LoggerCustom.getInstance().getRealTimestamp();
      LoggerCustom.getInstance().periodicBeforeUser();
      double userCodeStart = LoggerCustom.getInstance().getRealTimestamp();
      loopFunc();
      double loopCycleEnd = LoggerCustom.getInstance().getRealTimestamp();
      LoggerCustom.getInstance().recordOutput("LoggedRobot/FullCycleMS", (loopCycleEnd - loopCycleStart) * 1000);
      LoggerCustom.getInstance().recordOutput("LoggedRobot/LogPeriodicMS", (userCodeStart - loopCycleStart) * 1000);
      LoggerCustom.getInstance().recordOutput("LoggedRobot/UserCodeMS", (loopCycleEnd - userCodeStart) * 1000);

      LoggerCustom.getInstance().periodicAfterUser(); // Save data
    }
  }

  /** Ends the main loop in startCompetition(). */
  @Override
  public void endCompetition() {
    NotifierJNI.stopNotifier(notifier);
  }

  /** Get time period between calls to Periodic() functions. */
  public double getPeriod() {
    return period;
  }

  /** Sets whether to use standard timing or run as fast as possible. */
  public void setUseTiming(boolean useTiming) {
    this.useTiming = useTiming;
  }
}
