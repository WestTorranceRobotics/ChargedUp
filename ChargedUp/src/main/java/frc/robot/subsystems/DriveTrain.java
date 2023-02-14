// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;


public class DriveTrain extends SubsystemBase {

CANSparkMax leftLeader;
CANSparkMax rightLeader;
CANSparkMax leftFollower;
CANSparkMax rightFollower;

DifferentialDrive differentialDrive;



  /** Creates a new DriveTrain. */
  public DriveTrain() {

CANSparkMax leftLeader = new CANSparkMax(RobotMap.DriveTrainMap.leftLeaderCANID, MotorType.kBrushless);
CANSparkMax rightLeader = new CANSparkMax(RobotMap.DriveTrainMap.rightLeaderCANID, MotorType.kBrushless);
CANSparkMax leftFollower = new CANSparkMax(RobotMap.DriveTrainMap.leftFollowerCANID, MotorType.kBrushless);
CANSparkMax rightFollower = new CANSparkMax(RobotMap.DriveTrainMap.rightFollowerCANID, MotorType.kBrushless);

leftLeader.setIdleMode(IdleMode.kBrake);
rightLeader.setIdleMode(IdleMode.kBrake);
leftFollower.setIdleMode(IdleMode.kBrake);
rightFollower.setIdleMode(IdleMode.kBrake);
    
rightLeader.setInverted(true);
rightFollower.setInverted(true);

leftFollower.follow(leftLeader);
rightFollower.follow(rightLeader);

DifferentialDrive differentialDrive = new DifferentialDrive(leftLeader, rightLeader);



  }

  public void tankDrive(double leftPower, double rightPower){

differentialDrive.tankDrive(leftPower, rightPower);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
