// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.EnumMap;
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
  
  public static boolean enableDrivetrain = false;
  public static boolean enableClaw = true;
  public static boolean enableIntake = false;
  public static boolean enableLimelight = false;
  public static boolean enableSpindexer = false;
  public static boolean enableArm = false;
  public static boolean enableExtensionArm = false;
                                                                       
  
  public static int ads = 1;
  public static class SpindexerMap{
    public static int spinnerCANID = 0;
    public static double motorRevToRotation = 1/100;
    public static double spindexerSpeed = 0.75;
    public static boolean clockwise = true;
  }

  public final class ExtensionArmConstants{

    public static final int armMotorID =9;
    public static final double kP =  0.0125;
    public static final double kI = 0.000000125;
    public static final double kD = 0.065;
  }

  public final class ArmConstants{

    


    public static final int armMotorID =9;
    public static final double kP =  0.0125;
    public static final double kI = 0.000000125;
    public static final double kD = 0.065;
    
  
    //Default PID
    //0.0085
    //0.00000002
    // 0.036
  
  
    //Fast and far PID preset
    // 0.0125
    //0.000000125
    ///0.065
  }
  
  
  public static class ClawMap{

    //gear ratios: wheel turn: 10-1       limit switch: 2-1-10-1-80-18
    
    public static int controlModuleCANID = 0;
    
    public static int motionMotorCANID = 7;
    public static int powerMotorCANID = 11;
    
    public static int leftSolenoidPort = 0;
    public static int rightSolenoidPort = 0;
    
    public static int upLimitSwitchChannel = 5;
    public static int downLimitSwitchChannel = 6;
    
    public static double motionMotorFlipPower = 0.3;
    public static double motionMotorTurnPower = 0.2;
    public static double runClawPower = 0.5;
  }
  
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
  public static final int rightJoystickThreeButton = 3;
  public static final int rightJoystickFiveButton = 5;
  public static final int xboxControllerRightBack = 6;
  public static final int xboxControllerRightTrigger = 8;
}

}


/**

spinnerCANID
armMotorID
controlModuleCANID = 0;
    
 motionMotorCANID = 0;
 powerMotorCANID = 0;
    
   leftSolenoidPort = 0;
rightSolenoidPort = 0;
    
 upLimitSwitchChannel = 0;
 downLimitSwitchChannel = 0;
 intakeMotor1_ID
 intakeMotor2_ID

 leftLeader_ID = 1;
 leftFollower_ID = 2;
rightLeader_ID =3;
rightFollower_ID = 4;

rightLeaderEncoder1 = 4;
rightLeaderEncoder2 = 5;

 leftleaderEncoder1 = 2;

 leftleaderEncoder2 = 3;
 */