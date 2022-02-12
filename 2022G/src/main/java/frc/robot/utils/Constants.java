// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

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
  // Drivetrain constants
  public static final boolean DRIVE_USE_NORMALIZED_INPUTS = false;
  public static final boolean DRIVE_USE_SQUARED_INPUTS = false;

  public static final double DRIVING_DEADBANDS = 0.05;
  public static final double SPEED_MULTIPLIER = 1;
  public static final double TURN_MULTIPLIER = 1;

  // Hopper constants
  public static final int MAX_HOPPER_BELT_CURRENT = 15; // amps
  public static final int MAX_HOPPER_ROLLER_CURRENT = 15; // amps

  // Flywheel constants
  public static final double FLYWHEEL_P = 0.00015;
  public static final double FLYWHEEL_I = 0.0000002;
  public static final double FLYWHEEL_D = 0.0;
  public static final double FLYWHEEL_FF = 0.0002;
  public static final double FLYWHEEL_IZONE = 300;

  public static final double FLYWHEEL_RPM_FAR = 3500;
  public static final double FLYWHEEL_RPM_HIGH = 2500;
  public static final double FLYWHEEL_RPM_LOW = 2000;

  public static final double FLYWHEEL_THRESHOLD_HIGH = 50;
  public static final double FLYWHEEL_THRESHOLD_LOW = 50;
  public static final double FLYWHEEL_THRESHOLD_FAR = 50;

  public static final double FLYWHEEL_MAX_POWER = 1;
  public static final double FLYWHEEL_MAX_RPM = 4000;

  public static final double INTAKE_SPEED = 0.2;

  public static final double HOPPER_SPEED = 0.8;

  // OI constants
  public static final int XBOX_TRIGGER_SENSITIVITY = 0;
  public static final double XBOX_TRIGGER_DEADZONE = 0;

  public enum OIConfig {
    XBOX_TEST, JOYSTICK_TEST, COMPETITION
  }

  public static final double DRIVE_GEAR_RATIO = 6.0;
  public static final double CONVERT_INCHES_TO_METERS = 0.0254;
  public static final double DRIVE_WHEEL_DIAMETER = 4.0 * Constants.CONVERT_INCHES_TO_METERS;

  public static final double DRIVE_ENC_ROT_TO_DIST = (1 / Constants.DRIVE_GEAR_RATIO)
      * Math.PI
      * Constants.DRIVE_WHEEL_DIAMETER; // Encoder position conversion factor (native rotations ->
  // meters)

  // uncommment whichever one you want to use & comment the rest
  // public static final OIConfig OI_CONFIG = OIConfig.COMPETITION;
  public static final OIConfig OI_CONFIG = OIConfig.XBOX_TEST;
  // public static final OIConfig OI_CONFIG = OIConfig.JOYSTICK_TEST;

  // The Robot Characterization Toolsuite provides a convenient tool for obtaining
  // these
  // values for your robot.
  public static final double ksVolts = 0.13563;
  public static final double kvVoltSecondsPerMeter = 2.3576;
  public static final double kaVoltSecondsSquaredPerMeter = 0.21274;

  // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
  // These characterization values MUST be determined either experimentally or
  // theoretically
  // for *your* robot's drive.
  // Example value only - as above, this must be tuned for your drive!
  public static final double kPDriveVel = 0.00034928;
  // public static final double kPDriveVel = 0.015732;

  public static final double kTrackwidthMeters = 0.5842;
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

  public static final double kMaxSpeedMetersPerSecond = 1.7;
  public static final double kMaxAccelerationMetersPerSecondSquared = 1.7;

  // Reasonable baseline values for a RAMSET0E follower in units of meters and
  // seconds
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = .7;
}
