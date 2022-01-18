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
    public static final boolean DRIVE_USE_NORMALIZED_INPUTS = false;
    public static final boolean DRIVE_USE_SQUARED_INPUTS = true;

    public static final double DRIVING_DEADBANDS = 0.08;
    public static final double SPEED_MULTIPLIER = 0.2;
    public static final double TURN_MULTIPLIER = 0.08;
    public static final int MAX_TOWERBELT_SPEED = 15;
    public static final int MAX_TOWERROLLER_SPEED = 15;
    
  // Shooter constants
  public static final double FLYWHEEL_P = 0.00015;
  public static final double FLYWHEEL_I = 0.0000002;
  public static final double FLYWHEEL_D = 0.0;
  public static final double FLYWHEEL_FF = 0.0002;
  public static final double FLYWHEEL_IZONE = 300;

public static final double RPM_FAR = 0;
public static final double FLYWHEEL_THRESHOLD_LAYUP = 0;

public static final double FLYWHEEL_THRESHOLD_HIGH=0;
public static final double FLYWHEEL_THRESHOLD_LOW=0;
public static final double FLYWHEEL_THRESHOLD_FAR=0;
}

