// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;


public class DriveTrain extends SubsystemBase {

public WPI_TalonSRX leftLeader;
public WPI_TalonSRX rightLeader;
public WPI_TalonSRX leftFollower;
public WPI_TalonSRX rightFollower;

DifferentialDrive differentialDrive;

AHRS gyro;

  /** Creates a new DriveTrain. */
  public DriveTrain() {

WPI_TalonSRX leftLeader = new WPI_TalonSRX(RobotMap.DriveTrainMap.leftLeaderDeviceNumber);
WPI_TalonSRX rightLeader = new WPI_TalonSRX(RobotMap.DriveTrainMap.rightLeaderDeviceNumber);
WPI_TalonSRX leftFollower = new WPI_TalonSRX(RobotMap.DriveTrainMap.leftFollowerDeviceNumber);
WPI_TalonSRX rightFollower = new WPI_TalonSRX(RobotMap.DriveTrainMap.rightFollowerDeviceNumber);

leftLeader.setNeutralMode(NeutralMode.Brake);
rightLeader.setNeutralMode(NeutralMode.Brake);
leftFollower.setNeutralMode(NeutralMode.Brake);
rightFollower.setNeutralMode(NeutralMode.Brake);
    
rightLeader.setInverted(true);
rightFollower.setInverted(true);

leftFollower.follow(leftLeader);
rightFollower.follow(rightLeader);

DifferentialDrive differentialDrive = new DifferentialDrive(leftLeader, rightLeader);

AHRS gyro = new AHRS(SPI.Port.kMXP);

gyro.reset();
gyro.zeroYaw();

  }

  public void tankDrive(double leftPower, double rightPower){

differentialDrive.tankDrive(leftPower, rightPower);

  }

  public double getYaw(){

return gyro.getYaw();

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
