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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public final class ArmConstants{
    public static final int armMotorID =9;
    public static final double kP = 0.0085; //Fast Far preset 0.0125
    public static final double kI = 0.00000002; //0.000000125
    public static final double kD = 0.036; //0.065

    //Fast and far PID preset
    // 0.0125
    //0.000000125
    ///0.065
  }


  public final class IntakeConstants{
    public static final int intakeMotor1_ID = 6;
    public static final int intakeMotor2_ID = 7;
  }
  public final class ShooterConstants{
    public static final int shooterLeader_ID = 12;
    public static final int shooterFollower_ID = 7;
    public static final double kP = 0.00061675;
    public static final double kI = 0.00;
    public static final double kD =  0.52171675; 
  }

  public final class DriveTrainConstants{
    public static final int leftLeader_ID = 4;
    public static final int leftFollower_ID = 5;
    public static final int rightLeader_ID = 1;
    public static final int rightFollower_ID = 3;

    public static final int rightLeaderEncoder1 = 5;
    public static final int leftleaderEncoder1 = 6;
    public static final int rightFollowerEncoder1 = 7;
    public static final int leftFollowerEncoder1 = 8;
    public static final int rightLeaderEncoder2 = 9;
    public static final int leftleaderEncoder2 = 10;
    public static final int rightFollowerEncoder2 = 11;
    public static final int leftFollowerEncoder2 = 12;


}

public final class JoyStickConstants{
  public static final int leftJoystickPort = 0;
  public static final int rightJoystickPort = 1;
  public static final int xboxControllerPort = 2;
  public static final int leftJoystickTrigger = 1;
  public static final int rightJoystickTrigger = 1;
}


}
