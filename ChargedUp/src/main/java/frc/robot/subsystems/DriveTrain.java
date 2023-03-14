// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;


public class DriveTrain extends SubsystemBase {

  AHRS driveGyro;


  Encoder rightFollowerEncoder;
  Encoder rightLeaderEncoder;
  Encoder leftLeaderEncoder;
  Encoder leftFollowerEncoder;
  double speedPercentage;
  Solenoid clawsolenoid;
  
  DifferentialDrive drive;
  /** Creates a new DriveTrain. */
  PneumaticsControlModule pneumaticcontrolModule;
  WPI_TalonSRX rightLeader;
  WPI_TalonSRX rightFollower;
  WPI_TalonSRX leftFollower;
  WPI_TalonSRX leftLeader;
  PIDController gyroPID;
  double kP;
  double kI;
  double kD;
 
  Encoder rightEncoder = new Encoder(0,1,false,EncodingType.k2X);
  Encoder leftEncoder =  new Encoder(2,3,false,EncodingType.k2X);
  

  
  ShuffleboardTab drivesTab = Shuffleboard.getTab("DriveTab");  
  private GenericEntry SBLeftSpeed = drivesTab.add("Left Speed", 0).withPosition(0, 0).getEntry();
  private GenericEntry SBRightSpeed = drivesTab.add("Right Speed", 0).withPosition(1, 0).getEntry();
  private GenericEntry SBRightPosition =  drivesTab.add("Right Distance", 0).withPosition(2, 0).getEntry();
  private GenericEntry SBLeftPosition =  drivesTab.add("Left Distance", 0).withPosition(3, 0).getEntry();
  private GenericEntry SBSpeedPercentage = drivesTab.add("Speed Percentage", 50).withPosition(0,1).withSize(2, 1).getEntry();
  private GenericEntry SBGyroYaw = drivesTab.add("Gyro Yaw X",0).withPosition(0, 2).getEntry();
  private GenericEntry SBGyroPitch = drivesTab.add("Gyro Pitch",0).withPosition(1, 2).getEntry();
  private GenericEntry SBGyroRoll = drivesTab.add("Gyro Roll",0).withPosition(2, 2).getEntry();
  private GenericEntry SBGyroKp = drivesTab.add("Gyro kP",RobotMap.DriveTrainConstants.gyroPIDkP).withPosition(0, 3).getEntry();
  private GenericEntry SBGyroKi = drivesTab.add("Gyro kI",RobotMap.DriveTrainConstants.gyroPIDkI).withPosition(1, 3).getEntry();
  private GenericEntry SBGyroKd = drivesTab.add("Gyro kD",RobotMap.DriveTrainConstants.gyroPIDkD).withPosition(2, 3).getEntry();
 
  
    
  public DriveTrain() {
    
    //pneumaticcontrolModule = new PneumaticsControlModule(0);
    //clawsolenoid = new Solenoid(0,PneumaticsModuleType.CTREPCM, 0);

    kP = RobotMap.DriveTrainConstants.gyroPIDkP;
    kI = RobotMap.DriveTrainConstants.gyroPIDkI;
    kD = RobotMap.DriveTrainConstants.gyroPIDkD;
    driveGyro = new AHRS(SPI.Port.kMXP);
    driveGyro.zeroYaw();
    driveGyro.reset();
    speedPercentage = 50;
    gyroPID = new PIDController(kP, kI, kD);
    gyroPID.setTolerance(5.0);
    //Variables

    SBSpeedPercentage.setDouble(100);
    leftLeader = new WPI_TalonSRX(RobotMap.DriveTrainConstants.leftLeader_ID);
    leftFollower = new WPI_TalonSRX(RobotMap.DriveTrainConstants.leftFollower_ID);
    rightLeader = new WPI_TalonSRX(RobotMap.DriveTrainConstants.rightLeader_ID);
    rightFollower = new WPI_TalonSRX(RobotMap.DriveTrainConstants.rightFollower_ID);
    
    //no encoders on kitbot
    
    //dumblimitswtich2 = new DigitalInput(8);

// leftLeader.setSafetyEnabled(true);
// leftFollower.setSafetyEnabled(true);
// rightLeader.setSafetyEnabled(true);
// rightFollower.setSafetyEnabled(true);

    rightLeader.setInverted(true);

    drive = new DifferentialDrive(leftLeader,rightLeader);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftLeader.setInverted(false);
    leftFollower.setInverted(InvertType.FollowMaster);
    rightFollower.setInverted(InvertType.FollowMaster);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    leftLeader.setNeutralMode(NeutralMode.Brake);
    rightLeader.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);
      

  }

  public void resetEncoder(){
    //leftEncoder.reset();
    rightEncoder.reset();
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

  public void resetGyroPID(double p, double i , double d){
    gyroPID.setP(p);
    gyroPID.setI(i);
    gyroPID.setD(d);
    

  }

  public double getYaw(){
    return driveGyro.getYaw();
  }
  public double getRoll(){
    return driveGyro.getRoll();
  }

  public double getPitch(){
    return driveGyro.getPitch();
  }

  public double getRightDistance(){
    return rightEncoder.getDistance();
  }

  public double getLeftDistance(){
    return leftEncoder.getDistance();
  }

  public void gyroPIDDrive(){
    double calculation = MathUtil.clamp(gyroPID.calculate(getPitch(), 0), -0.75, 0.75);
    TankDrive(calculation, calculation);

  }

  public double getLeftVelocity(){
    //return leftLeaderEncoder.get();
    return 0.0;
  }
  public double getRightVelocity(){
    return rightEncoder.getDistance();
  }    
  

  @Override
  public void periodic() {

    //System.out.println("Right Encoder: " + rightEncoder.get());
    //SBDumbLimitSwitch2.setBoolean(dumblimitswtich2.get());

    if((kP != SBGyroKp.getDouble(0)) || (kI != SBGyroKi.getDouble(0)) || (kD != SBGyroKd.getDouble(0)) ){
      kP = SBGyroKp.getDouble(0);
      kI = SBGyroKi.getDouble(0);
      kD = SBGyroKd.getDouble(0);
      resetGyroPID(kP, kI, kD);
    }

    SBRightSpeed.setDouble(getRightVelocity());
    SBLeftSpeed.setDouble(getLeftVelocity());
    SBRightPosition.setDouble(getRightDistance());
    SBLeftPosition.setDouble(getLeftDistance());
   

    SBGyroYaw.setDouble(driveGyro.getYaw());
    SBGyroPitch.setDouble(driveGyro.getPitch());
    SBGyroRoll.setDouble(driveGyro.getRoll());

    //drivesTab.addDouble("HI", this::getRightVelocity);

    if (speedPercentage!= SBSpeedPercentage.getDouble(100)){
      SetSpeedPercentage(SBSpeedPercentage.getDouble(100));
    } 

    // This method will be called once per scheduler run
  }
}
