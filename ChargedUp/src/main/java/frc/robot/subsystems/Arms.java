// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;

import frc.robot.Robot;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.networktables.GenericEntry;


public class Arms extends SubsystemBase {
  private CANSparkMax armMotorController;

  private double targettedPowerVelocity;
  private double targettedPosition;
  private double targettedSetPoint;

  private ShuffleboardTab armTab = Shuffleboard.getTab("ArmTab");
  private GenericEntry SBArmCurrentSpeed = armTab.add("Arm Current Velocity", 0).withPosition(0, 0).getEntry();
  private GenericEntry SBArmCurrentPosition = armTab.add("Arm Current Position", 0).withPosition(1, 0).getEntry();
  
  private GenericEntry SBArmPIDP = armTab.add("PID P", RobotMap.ArmConstants.kP).withPosition(0, 1).getEntry();
  private GenericEntry SBArmPIDI = armTab.add("PID I", RobotMap.ArmConstants.kI).withPosition(1, 1).getEntry();
  private GenericEntry SBArmPIDD = armTab.add("PID D", RobotMap.ArmConstants.kD).withPosition(2, 1).getEntry();

  private GenericEntry SBArmTargettedPosition = armTab.add("Arm Targetted Position", 0).withPosition(0, 2).getEntry();
  private GenericEntry SBArmTargettedPowerSpeed = armTab.add("Arm Targetted Power Speed", 0.15).withPosition(1, 2).getEntry();
  private GenericEntry SBArmHeldPID = armTab.add("Hold PID?",0).withPosition(0, 3).getEntry();
  private GenericEntry SBResetArmPosition = armTab.add("Reset Position?",0).withPosition(1,3).getEntry();
  private GenericEntry SBCurrentArmSetPoint = armTab.add("Current Arm Setpoint",0).withPosition(0, 4).getEntry();
  private GenericEntry SBArmSetPointEnable = armTab.add("Enable Arm Set Point",0).withPosition(2, 3).getEntry();
  
  private GenericEntry PIDOffset = armTab.add("Arm Target Offset",0).withPosition(2, 3).getEntry();
  
  /** Creates a new Arms. */
  public Arms() {
    armMotorController = new CANSparkMax(RobotMap.ArmConstants.armMotorID, MotorType.kBrushless);
    armMotorController.restoreFactoryDefaults();
    armMotorController.getEncoder().setPosition(0);
    armMotorController.setInverted(false);
    armMotorController.setIdleMode(IdleMode.kBrake);

    targettedPosition = 0;
    targettedSetPoint = 1;
    targettedPowerVelocity = 0.15;
  }


  public void runArmPower(double Percentage){
    armMotorController.set(Percentage);
  }

  public void setTargttedPosition(double target){
    targettedPosition = target;
    SBArmTargettedPosition.setDouble(target);
  }

  public void toggleSetpoint(int toggling){
    SBArmSetPointEnable.setInteger(toggling);
  }
  public void togglePosition(int toggling){
    SBArmHeldPID.setInteger(toggling);
  }




  
  public void setSetPoint(int setpoint){
    targettedSetPoint = setpoint;
  }

  public double getTargettedSetPoint(){
    return targettedSetPoint; 
  }

  public void setSmallAnglePID(boolean toggle){
    if (toggle)
    {
      setPID(0.035, 0.0000125, 0.005);
    }
    else{
      setPID(0.02, 0.0000000125, 0.065);
    }

  }

  public void setPIDHiLo(){
    if(Math.abs(targettedPosition-armMotorController.getEncoder().getPosition()) < 20){
      setSmallAnglePID(true);
    }
    else{
      setSmallAnglePID(false);
    }
  }

  public void runSetPoint(){
    armMotorController.getPIDController().setOutputRange(-1, 1);

    if (targettedSetPoint ==0){
      setPIDHiLo();
      SBArmTargettedPosition.setDouble(0);
      targettedPosition = 0;
      armMotorController.getPIDController().setOutputRange(-0.5, 0.5);
    }

    else if (targettedSetPoint==1){
      setPIDHiLo();
      SBArmTargettedPosition.setDouble(-17.75);
      targettedPosition = -17.75;
    }

    else if (targettedSetPoint == 2){
      setPIDHiLo();
      SBArmTargettedPosition.setDouble(-53.2);
      targettedPosition = -53.2;
    }
    else if (targettedSetPoint ==3){
      setPIDHiLo();
      SBArmTargettedPosition.setDouble(-70);
      targettedPosition = -70;
    }

    if((targettedSetPoint ==0) && (armMotorController.getEncoder().getPosition()>=-1)){
      armMotorController.getEncoder().setPosition(0);

    }
  
    

    armMotorController.getPIDController().setReference(targettedPosition, ControlType.kPosition);
    
    if(Math.abs(targettedPosition-armMotorController.getEncoder().getPosition()) < 0.5){
      armMotorController.getPIDController().setIAccum(0);
    }

  }


  public void runArmPosition(double Position){
    armMotorController.getPIDController().setReference(Position, ControlType.kPosition);
  }

  public void increaseTargettedPosition(double increase){
    SBArmTargettedPosition.setDouble(SBArmTargettedPosition.getDouble(0)+increase);
    targettedPosition += increase;
  }


 public double getPosition(){
  return armMotorController.getEncoder().getPosition();
 } 

 public double getVelocity(){
  return armMotorController.getEncoder().getVelocity();
 }

 public void setPID(double p, double i, double d){
  SBArmPIDP.setDouble(p);
  SBArmPIDI.setDouble(i);
  SBArmPIDD.setDouble(d);

  armMotorController.getPIDController().setP(p);
  armMotorController.getPIDController().setI(i);
  armMotorController.getPIDController().setD(d);
 }

 public double getTargettedPowerVelocity(){
  return targettedPowerVelocity;
 }

 public double getTargettedPosition(){
  return targettedPosition;
 }

 public void resetTargettedPosition(){
  SBArmTargettedPosition.setDouble(armMotorController.getEncoder().getPosition());
 }






  
  

  @Override
  public void periodic() {
    PIDOffset.setDouble(targettedPosition - armMotorController.getEncoder().getPosition());

    if (SBResetArmPosition.getDouble(0) ==1){
      armMotorController.getEncoder().setPosition(0);
      SBResetArmPosition.setDouble(1);
    }

    if ((SBArmPIDD.getDouble(0)!= armMotorController.getPIDController().getP()) || (SBArmPIDI.getDouble(0)!= armMotorController.getPIDController().getI())||(SBArmPIDD.getDouble(0)!= armMotorController.getPIDController().getD(0))){
      setPID(SBArmPIDP.getDouble(0), SBArmPIDI.getDouble(0), SBArmPIDD.getDouble(0));
    }


    if (targettedPowerVelocity != SBArmTargettedPowerSpeed.getDouble(0)){
      targettedPowerVelocity = SBArmTargettedPowerSpeed.getDouble(0);
    }

    if(targettedPosition != SBArmTargettedPosition.getDouble(0)){
      targettedPosition = SBArmTargettedPosition.getDouble(0);
    }

    SBArmCurrentPosition.setDouble(armMotorController.getEncoder().getPosition());
    SBArmCurrentSpeed.setDouble(armMotorController.getEncoder().getVelocity());
    SBCurrentArmSetPoint.setDouble(getTargettedSetPoint());

    
    

    if(SBArmHeldPID.getDouble(0) == 1){
      runArmPosition(targettedPosition);
    }

    if(SBArmSetPointEnable.getDouble(0) == 1){
      runSetPoint();
    }
    



    // This method will be called once per scheduler run
  }
}
