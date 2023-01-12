// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveTrain extends SubsystemBase {
  TalonSRX leftLeader;
  TalonSRX leftFollower;
  TalonSRX rightLeader;
  TalonSRX rightFollower;

  Encoder rightFollowerEncoder;
  Encoder rightLeaderEncoder;
  Encoder leftLeaderEncoder;
  Encoder leftFollowerEncoder;
  double speedPercentage;
  
  DifferentialDrive drive;
  /** Creates a new DriveTrain. */

  private ShuffleboardTab drivesTab = Shuffleboard.getTab("DriveTab");  
  private GenericEntry SBLeftSpeed = drivesTab.add("Left Speed", 0).withPosition(0, 0).getEntry();
  private GenericEntry SBRightSpeed = drivesTab.add("Right Speed", 0).withPosition(1, 0).getEntry();
  private GenericEntry SBSpeedPercentage = drivesTab.add("Speed Percentage", 100).withPosition(0,1).withSize(2, 1).getEntry();

  
    
  public DriveTrain() {
    speedPercentage = 100;
    //Variables


    WPI_TalonSRX leftLeader = new WPI_TalonSRX(RobotMap.DriveTrain.leftLeader_ID);
    WPI_TalonSRX leftFollower = new WPI_TalonSRX(RobotMap.DriveTrain.leftFollower_ID);
    WPI_TalonSRX righttLeader = new WPI_TalonSRX(RobotMap.DriveTrain.rightLeader_ID);
    WPI_TalonSRX righttFollower = new WPI_TalonSRX(RobotMap.DriveTrain.rightFollower_ID);
    
    Encoder rightEncoder = new Encoder(RobotMap.DriveTrain.rightLeaderEncoder1,RobotMap.DriveTrain.rightLeaderEncoder2,false);
    Encoder leftEncoder = new Encoder(RobotMap.DriveTrain.leftleaderEncoder1,RobotMap.DriveTrain.leftleaderEncoder2,false);


    



    drive = new DifferentialDrive(righttLeader,leftLeader);

    leftFollower.follow(leftLeader);
    righttFollower.follow(righttLeader);

    leftLeader.setInverted(true);
    leftFollower.setInverted(InvertType.FollowMaster);
    righttFollower.setInverted(InvertType.FollowMaster);
    

  }



  public void arcadeDrive(double InputSpeed, double InputRotation){
    drive.arcadeDrive(InputSpeed*speedPercentage, InputRotation);
  }

  public void TankDrive(double InputLeftSpeed, double InputRightSpeed){
    drive.tankDrive(InputLeftSpeed*speedPercentage, InputRightSpeed*speedPercentage);
  }

  public int GetLeftSpeed(){
    return leftLeaderEncoder.get();
  }



public void UpdateShuffleBoard(){
  if (speedPercentage != SBSpeedPercentage.getDouble(100)){
    speedPercentage = SBSpeedPercentage.getDouble(100);
  }
  
  SBLeftSpeed.setInteger(leftLeaderEncoder.get());
  SBRightSpeed.setInteger(rightLeaderEncoder.get());
  

}  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
