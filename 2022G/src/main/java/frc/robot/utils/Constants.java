// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Is Gullinkambi
  public static final boolean IS_GULLINKAMBI = true;
  
//uncommment whichever one you want to use & comment the rest
  //public static final OIConfig OI_CONFIG = OIConfig.COMPETITION; 
  //public static final OIConfig OI_CONFIG = OIConfig.XBOX_TEST;
  public static final OIConfig OI_CONFIG = OIConfig.JOYSTICK_TEST;


  // Drivetrain constants
  public static final int DRIVETRAIN_MAX_CURRENT = 40;

  public static final boolean DRIVE_USE_NORMALIZED_INPUTS = false;
  public static final boolean DRIVE_USE_SQUARED_INPUTS = false;

  public static final double DRIVING_DEADBANDS = 0.1;
  public static final double SPEED_MULTIPLIER = 1;
  public static final double TURN_MULTIPLIER = 1;

  // Intake constants
  public static final int INTAKE_MAX_CURRENT = 30; // amps
  public static final double INTAKE_SPEED = 0.5;

  // Hopper constants
  public static final int HOPPER_MAX_CURRENT = 30; // amps
  public static final double HOPPER_SPEED = 0.5;
  public static final double LOWER_SENSOR_INPUT_THRESHOLD = 0.99;

  // Flywheel constants
  public static final int FLYWHEEL_MAX_CURRENT = 40; // amps

  public static final double FLYWHEEL_P = 0.0002;
  public static final double FLYWHEEL_I = 0.000001;
  public static final double FLYWHEEL_D = 0.0001;
  public static final double FLYWHEEL_FF = 0.000186;
  public static final double FLYWHEEL_IZONE = 100;

  public static final double FLYWHEEL_RPM_SHOOTLL = 2800;
  public static final double FLYWHEEL_RPM_LAYUP = 2400;
  public static final double FLYWHEEL_RPM_LOW = 1500;
  public static final double FLYWHEEL_RPM_REV_UP = 1500;

  public static final double FLYWHEEL_THRESHOLD_LAYUP = 100;
  public static final double FLYWHEEL_THRESHOLD_LOW = 100;
  public static final double FLYWHEEL_THRESHOLD_SHOOTLL = 100;

  public static final double FLYWHEEL_MAX_POWER = 1;
  public static final double FLYWHEEL_MAX_RPM = 4000;

  // Climber constants
  public static final double CLIMBER_TOP_ENCODER_POSITION = -116;
  public static final double CLIMBER_P = 0.3;
  public static final double CLIMBER_I = 0.00001;
  public static final double CLIMBER_D = 0.0;
  public static final double CLIMBER_FF = 0.0;
  public static final double CLIMBER_IZONE = 10;

  public static final int CLIMBER_MAX_CURRENT = 30;

  public static final double LL_P = -0.0033;
  public static final double LL_I = 0.0000001;
  public static final double LL_D = 0;
  public static final double LL_ANGLE = 20;
  public static final double LL_HEIGHT = 24.5;
  public static final double LL_PANNING = 0;
  public static final double TARGET_HEIGHT = 104;
  
  public static final double [] LL_DIST = new double[]{100,150,200};
  public static final double [] RPMS = new double[]{2800,3000,3200};
  public static LookupTable DIST_TO_RPM = new LookupTable(LL_DIST, RPMS);

  // OI constants
  public static final int XBOX_TRIGGER_SENSITIVITY = 0;
  public static final double XBOX_TRIGGER_DEADZONE = 0;

  public enum OIConfig {
    XBOX_TEST, JOYSTICK_TEST, COMPETITION
  }

  public static final double DRIVE_GEAR_RATIO = 7.3688888;
  public static final double CONVERT_INCHES_TO_METERS = 0.0254;
  public static final double DRIVE_WHEEL_DIAMETER = 4.0 * Constants.CONVERT_INCHES_TO_METERS;

  public static final double DRIVE_ENC_ROT_TO_DIST = (1 / Constants.DRIVE_GEAR_RATIO)
      * Math.PI
      * Constants.DRIVE_WHEEL_DIAMETER; // Encoder position conversion factor (native rotations ->
  // meters)

    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.13563;
    public static final double kvVoltSecondsPerMeter = 2.3576;
    public static final double kaVoltSecondsSquaredPerMeter = 0.21274;


      // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // Example value only - as above, this must be tuned for your drive!
    // public static final double kPDriveVel = 0.00034928;
    //public static final double kPDriveVel = SmartDashboard.getNumber("KPDriveVel", 0);
    public static final double kPDriveVel = 0.00034928;

    public static final double kTrackwidthMeters = 0.5842;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 1.2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.2;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = .7;
}
