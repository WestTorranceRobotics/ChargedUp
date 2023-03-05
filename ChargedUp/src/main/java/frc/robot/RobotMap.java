// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.HashMap;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

import edu.wpi.first.wpilibj2.command.Subsystem;

public final class RobotMap {
  public static final HashMap<String, Boolean> enable = new HashMap<String, Boolean>();



  

  


  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public final class IntakeConstants{
    public static final int intakeMotor1_ID = 6;
    public static final int intakeMotor2_ID = 7;
  }

  public final class DriveTrainConstants{
    public static final int leftLeader_ID = 1;
    public static final int leftFollower_ID = 2;
    public static final int rightLeader_ID =3;
    public static final int rightFollower_ID = 4;

    public static final int rightLeaderEncoder1 = 4;
    public static final int rightLeaderEncoder2 = 5;

    public static final int leftleaderEncoder1 = 2;
    public static final int leftleaderEncoder2 = 3;
    
    public static final double gyroPIDkP = 1;
    public static final double gyroPIDkI = 0;
    public static final double gyroPIDkD = 0;




}

public final class JoyStickConstants{
  public static final int leftJoystickPort = 0;
  public static final int rightJoystickPort = 1;
  public static final int xboxControllerPort = 2;
  public static final int leftJoystickTrigger = 1;
  public static final int rightJoystickTrigger = 1;
}


}
