package frc.robot.utils;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;


public final class Constants {
  
  //uncommment whichever one you want to use & comment the rest
  public static final OIConfig OI_CONFIG = OIConfig.COMPETITION; 
  // public static final OIConfig OI_CONFIG = OIConfig.XBOX_TEST;
  //public static final OIConfig OI_CONFIG = OIConfig.JOYSTICK_TEST;
  // public static final OIConfig OI_CONFIG = OIConfig.PS5_TEST;

  // Drivetrain constants
  public static final int DRIVETRAIN_MAX_CURRENT = 40;

  public static final boolean DRIVE_USE_NORMALIZED_INPUTS = false;
  public static final boolean DRIVE_USE_SQUARED_INPUTS = false;

  public static final double DRIVING_DEADBANDS = 0.15;
  public static final double SPEED_MULTIPLIER = 1.5;
  public static final double TURN_MULTIPLIER = 1;
  public static final double DRIVETRAIN_CLOSEDLOOP_RAMPRATE = 0.1;
  public static final double DRIVETRAIN_OPENLOOP_RAMPRATE = 0.1;

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
  public static final double HOPPER_SHOOT_LAYUP_SPEED = -1800; // rpm
  public static final double HOPPER_SHOOT_LL_SPEED = -3800; // rpm
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

  public static final double FLYWHEEL_P = 0.00002;
  
  public static final double FLYWHEEL_I = 0.0000001;
  public static final double FLYWHEEL_D = 0.0;
  public static final double FLYWHEEL_FF = 0.0;
  public static final double FLYWHEEL_IZONE = 60;

  public static final double FLYWHEEL_RPM_LAYUP = 2650;
  public static final double FLYWHEEL_RPM_LOW = 1300;
  public static final double FLYWHEEL_RPM_REV_UP_STANDARD = 2800;

  public static final double FLYWHEEL_THRESHOLD_LAYUP = 50;
  public static final double FLYWHEEL_THRESHOLD_LOW = 50;
  public static final double FLYWHEEL_THRESHOLD_SHOOTLL = 50;

  public static final double FLYWHEEL_MAX_POWER = 1;
  public static final double FLYWHEEL_MAX_RPM = 4500;

  public static final double ksFlywheel = 0.24292*0.97;
  public static final double kvFlywheel = 0.12977*0.97;
  public static final double kaFlywheel = 0.0053723*0.97;

  // Climber constants
  public static final double CLIMBER_TOP_ENCODER_POSITION = -116;
  public static final double CLIMBER_P = 0.3;
  public static final double CLIMBER_I = 0.00001;
  public static final double CLIMBER_D = 0.0;
  public static final double CLIMBER_FF = 0.0;
  public static final double CLIMBER_IZONE = 10;

  public static final int CLIMBER_MAX_CURRENT = 60;

  // Limelight constants
  public static final double LL_P = 0.006;
  public static final double LL_I = 0;
  public static final double LL_D = 0.0;
  public static final double LL_FF = 0.15;
  public static final double LL_ANGLE_BOUND = 1.0;
  public static final double LL_ANGLE = 45; 
  public static final double LL_HEIGHT = 24.5;
  public static final double LL_PANNING_ANGLE = 0;
  public static final double TARGET_HEIGHT = 104;
  
  public static final double [] LL_DIST = new double[]{62, 73, 90, 105, 120, 135, 150, 170, 190};
  public static final double i = 200;
  public static final double [] RPMS = new double[]{2200+i, 2270+i, 2410+i, 2520+i, 2700+i, 2800+i, 3020+i, 3340+i, 3590+i};
  public static LookupTable DIST_TO_RPM = new LookupTable(LL_DIST, RPMS);

  // OI constants
  public static final int XBOX_TRIGGER_SENSITIVITY = 0;
  public static final double XBOX_TRIGGER_DEADZONE = 0;

  public enum OIConfig {
    XBOX_TEST, JOYSTICK_TEST, COMPETITION, PS5_TEST
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

    public static final double kPDriveVel = 0.25;
    
    public static final double kTrackwidthMeters = 0.72791; /* True calculations: 0.653*/
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = .7;

    
}
