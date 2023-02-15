// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class RobotMap {

  public static class ClawMap{

//gear ratios: wheel turn: 10-1       limit switch: 2-1-10-1-80-18

public static int controlModuleCANID = 0;

public static int motionMotorCANID = 0;
public static int powerMotorCANID = 0;

public static int leftSolenoidPort = 0;
public static int rightSolenoidPort = 0;

public static int upLimitSwitchChannel = 0;
public static int downLimitSwitchChannel = 0;

public static double motionMotorFlipPower = 0.3;
public static double motionMotorTurnPower = 0.2;
public static double runClawPower = 0.5;

  }
  public static class OperatorConstants {
   
  }
}
