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
import edu.wpi.first.wpilibj.Timer;
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
  PIDController orientationPID;
  PIDController distancePID;

  double kP;
  double kI;
  double kD;

  double d_kP;
  double d_kI;
  double d_kD;

  double a_kP;
  double a_kI;
  double a_kD;

  double startYawOffset;
  
 
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
  private GenericEntry SBDistanceKp = drivesTab.add("Distance kP",RobotMap.DriveTrainConstants.distancePIDkP).withPosition(0, 4).getEntry();
  private GenericEntry SBDistanceKi = drivesTab.add("Distance kI",RobotMap.DriveTrainConstants.distancePIDkI).withPosition(1, 4).getEntry();
  private GenericEntry SBDistanceKd = drivesTab.add("Distance kD",RobotMap.DriveTrainConstants.distancePIDkD).withPosition(2, 4).getEntry();
  private GenericEntry SBtargetPosition = drivesTab.add("Target position",0).withPosition(4, 0).getEntry();
  private GenericEntry SBEnablePIDPosition = drivesTab.add("Enable position",0).withPosition(5, 0).getEntry();
  private GenericEntry SBRotationKp = drivesTab.add("Rotation kP",RobotMap.DriveTrainConstants.distancePIDkP).withPosition(0, 6).getEntry();
  private GenericEntry SBRotationKi = drivesTab.add("Rotation kI",RobotMap.DriveTrainConstants.distancePIDkI).withPosition(1, 6).getEntry();
  private GenericEntry SBRotationKd = drivesTab.add("Rotation kD",RobotMap.DriveTrainConstants.distancePIDkD).withPosition(2, 6).getEntry();
  private GenericEntry SBtargetAngle = drivesTab.add("Target angle",0).withPosition(3, 6).getEntry();
  private GenericEntry SBResetPosition =  drivesTab.add("Reset Position",0).withPosition(7, 0).getEntry();
  private GenericEntry SBstartingGyro =  drivesTab.add("Starting Pitch",0).withPosition(4, 0).getEntry();
  private GenericEntry SBstartingYaw =  drivesTab.add("Starting Yaw",0).withPosition(5, 0).getEntry();
  private GenericEntry SBtargetSpeed =  drivesTab.add("Target speed",0).withPosition(6, 0).getEntry();

  private GenericEntry SB005 = drivesTab.add("0.05 Delay",0).withPosition(1, 4).getEntry();
  private GenericEntry SB01 = drivesTab.add("0.1 Delay",0).withPosition(2, 4).getEntry();
  Timer timer = new Timer();
  

  public DriveTrain() {
    
    //pneumaticcontrolModule = new PneumaticsControlModule(0);
    //clawsolenoid = new Solenoid(0,PneumaticsModuleType.CTREPCM, 0);

    kP = RobotMap.DriveTrainConstants.gyroPIDkP;
    kI = RobotMap.DriveTrainConstants.gyroPIDkI;
    kD = RobotMap.DriveTrainConstants.gyroPIDkD;

    d_kP = RobotMap.DriveTrainConstants.distancePIDkP;
    d_kI = RobotMap.DriveTrainConstants.distancePIDkI;
    d_kD = RobotMap.DriveTrainConstants.distancePIDkD;

    
    

    
    driveGyro = new AHRS(SPI.Port.kMXP);
    driveGyro.calibrate();
    driveGyro.zeroYaw();
    driveGyro.reset();
    speedPercentage = 100;
    gyroPID = new PIDController(0.02, 0, 0.003);
    distancePID = new PIDController(d_kP, d_kI, d_kD);
    orientationPID = new PIDController(0.02, 0, 0.003);
    orientationPID.setTolerance(1);
    gyroPID.setTolerance(1.0);

    

    startYawOffset = driveGyro.getYaw();
    
    // gyroPID = new PIDController(kP, kI, kD);
    // gyroPID.setTolerance(5.0);

    // gyroPID = new PIDController(kP, kI, kD);
    // gyroPID.setTolerance(5.0);
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

    drive = new DifferentialDrive(leftLeader,rightLeader);
    drive.setSafetyEnabled(true);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftLeader.setInverted(false);
    leftFollower.setInverted(false);
    
    rightLeader.setInverted(true);
    rightFollower.setInverted(true);

    leftFollower.setNeutralMode(NeutralMode.Brake);
    leftLeader.setNeutralMode(NeutralMode.Brake);
    rightLeader.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);

    rightEncoder.setReverseDirection(true);
    leftEncoder.setReverseDirection(false);
  }

  public void resetEncoders(){
    leftEncoder.reset();
    rightEncoder.reset();
  }
  public void restartYaw(){
    SBstartingYaw.setDouble(driveGyro.getYaw());
    startYawOffset = driveGyro.getYaw();

  }

  public double getStartYawOffset(){
    return SBstartingYaw.getDouble(0);
  }
  public void displayPitch(double pitch){
    SBstartingGyro.setDouble(pitch);
  }

  public void arcadeDrive(double InputSpeed, double InputRotation){
    //if (0.2<InputSpeed){
      SBtargetSpeed.setDouble(InputSpeed*(speedPercentage/100));
      drive.arcadeDrive(InputSpeed*(speedPercentage/100), InputRotation*0.7);
    
  }

  public void TankDrive(double InputLeftSpeed, double InputRightSpeed){
    leftLeader.set(ControlMode.PercentOutput, InputLeftSpeed*(speedPercentage/100));
    leftFollower.set(ControlMode.PercentOutput, InputLeftSpeed*(speedPercentage/100));

    rightLeader.set(ControlMode.PercentOutput, InputRightSpeed*(speedPercentage/100));
    rightFollower.set(ControlMode.PercentOutput, InputRightSpeed*(speedPercentage/100));

    // drive.tankDrive(InputLeftSpeed*(speedPercentage/100), InputRightSpeed*(speedPercentage/100));
  }

  public void TankDriveStraight(double speed, double totalDist){
    double error;
    double weight = 1;

    if(getRightDistance() < getLeftDistance()){
      error = (getLeftDistance()-getRightDistance())/totalDist;
      TankDrive(speed-(error*weight*Math.signum(speed)), speed);
    }
    else{
      error = (getRightDistance()-getLeftDistance())/totalDist;
      TankDrive(speed, speed-(error*weight*Math.signum(speed)));
    }
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

  public PIDController GetOrientationPID()
  {
    return orientationPID;
  }

  public void resetDistancePID(double p, double i, double d){
    distancePID.setP(p);
    distancePID.setI(i);
    distancePID.setD(d);
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

  public void gyroPIDDrive(double setpoint){
    double calculation = MathUtil.clamp(gyroPID.calculate(getPitch(),setpoint),-0.75, 0.75);
    TankDrive(calculation, calculation);

  }

  public void distancePIDDrive(double setpoint){
    //double error;
    distancePID.setTolerance(10);
    //double weight = 1;
    double calculate = MathUtil.clamp(distancePID.calculate(getRightDistance(), setpoint), -0.75, 0.75);

    // if(getRightDistance() < getLeftDistance()){
    //   error = (getLeftDistance()-getRightDistance())/setpoint;
    //   TankDrive(calculate-(error*weight*Math.ssignum(calculate)), calculate);
    // }
    // else{
    //   error = (getRightDistance()-getLeftDistance())/setpoint;
    //   TankDrive(calculate, calculate-(error*weight*Math.signum(calculate)));
    // }
    TankDrive(calculate, calculate);
    
  }

  public void resetDistance(){
    rightEncoder.reset();
  }

  public AHRS getGyro(){
    return driveGyro;
  }

  public double getLeftVelocity(){
    //return leftLeaderEncoder.get();
    return 0.0;
  }
  public double getRightVelocity(){
    return rightEncoder.getDistance();
  }    
  


  public PIDController getGyroPID(){
    return gyroPID;
  }

  public void resetGyro(){
    driveGyro.reset();
  }

  public boolean getIsCalibrating(){
    return driveGyro.isCalibrating();
  }

  double counter = 0;
  @Override
  public void periodic() {
    // if(counter <=5 ){
    //   startYawOffset = driveGyro.getYaw();
    //   counter += 1;
    // }
    // if (SBEnablePIDPosition.getDouble(0)==1 && Math.abs(getLeftDistance() - SBtargetPosition.getDouble(a_kD))>20){
    //   distancePIDDrive();
    // }

    //System.out.println("Right Encoder: " + rightEncoder.get());
    //SBDumbLimitSwitch2.setBoolean(dumblimitswtich2.get());

    // if((kP != SBGyroKp.getDouble(0)) || (kI != SBGyroKi.getDouble(0)) || (kD != SBGyroKd.getDouble(0)) ){
    //   kP = SBGyroKp.getDouble(0);
    //   kI = SBGyroKi.getDouble(0);
    //   kD = SBGyroKd.getDouble(0);
    //   resetGyroPID(kP, kI, kD);
    // }

     if((d_kP != SBDistanceKp.getDouble(0)) || (d_kI != SBDistanceKi.getDouble(0)) || (d_kD != SBDistanceKd.getDouble(0)) ){
      d_kP = SBDistanceKp.getDouble(0);
       d_kI = SBDistanceKi.getDouble(0);
       d_kD = SBDistanceKd.getDouble(0);
       resetDistancePID(d_kP, d_kI, d_kD);
     }

    if ((a_kP != SBRotationKp.getDouble(0)) || (a_kI != SBRotationKi.getDouble(0)) || (a_kD != SBRotationKd.getDouble(0))){
      a_kP = SBRotationKp.getDouble(0);
      a_kI = SBRotationKi.getDouble(0);
      a_kD = SBRotationKd.getDouble(0);
      distancePID.setPID(a_kP, a_kI, a_kD);
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

    if (SBResetPosition.getInteger(0) ==1){
      leftEncoder.reset();
      rightEncoder.reset();

    }
    if (SBEnablePIDPosition.getInteger(0)==1){
      distancePIDDrive(0);
    }

    // This method will be called once per scheduler run
  }
}
