package frc.robot.utils;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;


public final class Constants {
  // use logging?
  public static final boolean USE_LOGGING = false;
  
  //uncommment whichever one you want to use & comment the rest
  public static final OIConfig OI_CONFIG = OIConfig.COMPETITION; 
  // public static final OIConfig OI_CONFIG = OIConfig.XBOX_TEST;
  //public static final OIConfig OI_CONFIG = OIConfig.JOYSTICK_TEST;

  // Drivetrain constants
  public static final int DRIVETRAIN_MAX_CURRENT = 40;

  public static final boolean DRIVE_USE_NORMALIZED_INPUTS = false;
  public static final boolean DRIVE_USE_SQUARED_INPUTS = false;

  public static final double DRIVING_DEADBANDS = 0.1;
  public static final double SPEED_MULTIPLIER = 1;
  public static final double TURN_MULTIPLIER = 1;

  public static final double kTurnToAngleP = 0.0055;
  public static final double kTurnToAngleI = 0.0000005;
  public static final double kTurnToAngleD = 0.0;
  public static final double kTurnToAngleFF = 0.15;
  public static final double kTurnToAngleToleranceDeg = 0.5;
  public static final double kTurnToAngleRateToleranceDegPerS = 0;

  // Intake constants
  public static final int INTAKE_MAX_CURRENT = 30; // amps
  public static final double INTAKE_SPEED = 1.0;

  // Hopper constants
  public static final int HOPPER_MAX_CURRENT = 30; // amps
  public static final double HOPPER_SHOOT_LAYUP_SPEED = -3000; // rpm
  public static final double HOPPER_SHOOT_LL_SPEED = -3000; // rpm
  public static final double HOPPER_SHOOT_POWER = 0.65;
  public static final double HOPPER_INDEX_POWER = 0.7;
  public static final double LOWER_SENSOR_INPUT_THRESHOLD = 0.99;
  public static final double UPPER_SENSOR_INPUT_THRESHOLD = 0.6;

  public static final double HOPPER_VEL_P = 0.00002;
  public static final double HOPPER_VEL_I = 0.0000002;
  public static final double HOPPER_VEL_D = 0.0;
  public static final double HOPPER_VEL_FF = 0.00014;

  // Flywheel constants
  public static final int FLYWHEEL_MAX_CURRENT = 40; // amps

  public static final double FLYWHEEL_P = 0.0;
  public static final double FLYWHEEL_I = 0.0;
  public static final double FLYWHEEL_D = 0.0;
  public static final double FLYWHEEL_FF = 0.0;
  public static final double FLYWHEEL_IZONE = 100;

  public static final double ksFlywheel = 0.0;
  public static final double kvFlywheel = 0.0;
  public static final double kaFlywheel = 0.0;

  public static final double FLYWHEEL_RPM_LAYUP = 2500;
  public static final double FLYWHEEL_RPM_LOW = 1300;
  public static final double FLYWHEEL_RPM_REV_UP = 1500;

  public static final double FLYWHEEL_THRESHOLD_LAYUP = 50;
  public static final double FLYWHEEL_THRESHOLD_LOW = 50;
  public static final double FLYWHEEL_THRESHOLD_SHOOTLL = 50;

  public static final double FLYWHEEL_MAX_POWER = 1;
  public static final double FLYWHEEL_MAX_RPM = 4000;

  // Climber constants
  public static final double CLIMBER_TOP_ENCODER_POSITION = -116;
  public static final double CLIMBER_P = 0.3;
  public static final double CLIMBER_I = 0.00001;
  public static final double CLIMBER_D = 0.0;
  public static final double CLIMBER_FF = 0.0;
  public static final double CLIMBER_IZONE = 10;

  public static final int CLIMBER_MAX_CURRENT = 60;

  // Limelight constants
  public static final double LL_P = 0.01;
  public static final double LL_I = 0;
  public static final double LL_D = 0;
  public static final double LL_FF = 0.15;
  public static final double LL_ANGLE_BOUND = 1.0;
  public static final double LL_ANGLE = 45; 
  public static final double LL_HEIGHT = 24.5;
  public static final double LL_PANNING_ANGLE = 0;
  public static final double TARGET_HEIGHT = 104;
  
  public static final double [] LL_DIST = new double[]{63, 68, 90, 104, 120, 135, 150, 178};
  public static final double [] RPMS = new double[]{2371, 2471, 2625, 2710, 2830, 3015, 3160, 3590};
  public static LookupTable DIST_TO_RPM = new LookupTable(LL_DIST, RPMS);

  // OI constants
  public static final int XBOX_TRIGGER_SENSITIVITY = 0;
  public static final double XBOX_TRIGGER_DEADZONE = 0;

  public enum OIConfig {
    XBOX_TEST, JOYSTICK_TEST, COMPETITION
  }

  public static final double DRIVE_GEAR_RATIO = 7.6388888;
  public static final double CONVERT_INCHES_TO_METERS = 0.0254;
  public static final double DRIVE_WHEEL_DIAMETER = 3.95 * Constants.CONVERT_INCHES_TO_METERS;

  public static final double DRIVE_ENC_ROT_TO_DIST = (1 / Constants.DRIVE_GEAR_RATIO)
      * Math.PI
      * Constants.DRIVE_WHEEL_DIAMETER; // Encoder position conversion factor (native rotations ->
  // meters)

    // Autonomous constants
    public static final double ksVoltsLeft = 0.13408;
    public static final double kvVoltSecondsPerMeterLeft = 3.1544;
    public static final double kaVoltSecondsSquaredPerMeterLeft = 0.13889;

    public static final double ksVoltsRight = 0.1255;
    public static final double kvVoltSecondsPerMeterRight = 3.1233;
    public static final double kaVoltSecondsSquaredPerMeterRight = 0.13889;

    public static final double kPDriveVel = 0.25; //0.5; //0.0013735; // 0.03125; //0.03125; //1.4228E-05; //0.00020646; // 0.03125;
    
    public static final double kTrackwidthMeters = 0.72791; /* True calculations: 0.653*/
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = .7;
}
