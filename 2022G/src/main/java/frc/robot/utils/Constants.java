// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants { 
  // Drivetrain constants
  public static final boolean DRIVE_USE_NORMALIZED_INPUTS = false;
  public static final boolean DRIVE_USE_SQUARED_INPUTS = true;

  public static final double DRIVING_DEADBANDS = 0.08;
  public static final double SPEED_MULTIPLIER = 0.2;
  public static final double TURN_MULTIPLIER = 0.2;
  public static final double SLOW_SPEED_MULTIPLIER = 0.6;
  public static final double SLOW_TURN_MULTIPLIER = 0.7;

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

  public static final double FLYWHEEL_MAX_POWER = 0.5;
  public static final double FLYWHEEL_MAX_RPM = 4000;


  // OI constants
  public static final int XBOX_TRIGGER_SENSITIVITY = 0;
  public static final double XBOX_TRIGGER_DEADZONE = 0;
  public enum OIConfig {
    XBOX_TEST, JOYSTICK_TEST, COMPETITION
}

//uncommment whichever one you want to use & comment the rest
//public static final OIConfig OI_CONFIG = OIConfig.COMPETITION; 
public static final OIConfig OI_CONFIG = OIConfig.XBOX_TEST; 
//public static final OIConfig OI_CONFIG = OIConfig.JOYSTICK_TEST;
  
}

