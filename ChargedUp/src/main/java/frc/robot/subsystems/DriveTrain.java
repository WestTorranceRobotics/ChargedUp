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
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveTrain extends SubsystemBase {

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

    SBSpeedPercentage.setDouble(100);
    WPI_TalonSRX leftLeader = new WPI_TalonSRX(RobotMap.DriveTrainConstants.leftLeader_ID);
    WPI_TalonSRX leftFollower = new WPI_TalonSRX(RobotMap.DriveTrainConstants.leftFollower_ID);
    WPI_TalonSRX rightLeader = new WPI_TalonSRX(RobotMap.DriveTrainConstants.rightLeader_ID);
    WPI_TalonSRX rightFollower = new WPI_TalonSRX(RobotMap.DriveTrainConstants.rightFollower_ID);
    
    //no encoders on kitbot
    //Encoder rightEncoder = new Encoder(RobotMap.DriveTrainConstants.rightLeaderEncoder1,RobotMap.DriveTrainConstants.rightLeaderEncoder2,false);
    //Encoder leftEncoder = new Encoder(RobotMap.DriveTrainConstants.leftleaderEncoder1,RobotMap.DriveTrainConstants.leftleaderEncoder2,false);

leftLeader.setSafetyEnabled(true);
leftFollower.setSafetyEnabled(true);
rightLeader.setSafetyEnabled(true);
rightFollower.setSafetyEnabled(true);



    drive = new DifferentialDrive(leftLeader,rightLeader);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftLeader.setInverted(true);
    leftFollower.setInverted(InvertType.FollowMaster);
    rightFollower.setInverted(InvertType.FollowMaster);
    

  }



  public void arcadeDrive(double InputSpeed, double InputRotation){
    //if (0.2<InputSpeed){

      drive.arcadeDrive(InputSpeed*(speedPercentage/100), InputRotation*0.7);
    
  }

  public void TankDrive(double InputLeftSpeed, double InputRightSpeed){
    drive.tankDrive(InputLeftSpeed*(speedPercentage/100), InputRightSpeed*(speedPercentage/100));
  }

  public int GetLeftSpeed(){
    return leftLeaderEncoder.get();
  }

  public void StopDrive(){
    drive.tankDrive(0, 0);
  }

  public void SetSpeedPercentage(double percent){
    speedPercentage = percent;
  }




  
  

  

  @Override
  public void periodic() {
   if (speedPercentage!= SBSpeedPercentage.getDouble(100)){
    SetSpeedPercentage(SBSpeedPercentage.getDouble(100));
   } 
    // This method will be called once per scheduler run
  }
}
