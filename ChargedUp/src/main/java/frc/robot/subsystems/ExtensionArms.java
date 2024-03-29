// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.lang.model.element.Element;

import com.revrobotics.CANSparkMax;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.AutoCommands.HelperCommands.Delay;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ExtensionArms extends SubsystemBase {
  
  private CANSparkMax m_extensionarm;
  private double targettedPowerVelocity;
  private double targettedPosition;
  private double targettedSetPoint;
  
  

  private ShuffleboardTab extensionarmTab = Shuffleboard.getTab("Extension Arm");
  private GenericEntry SBArmCurrentSpeed = extensionarmTab.add("Arm Current Velocity", 0).withPosition(0, 0).getEntry();
  private GenericEntry SBArmCurrentPosition = extensionarmTab.add("Arm Current Position", 0).withPosition(1, 0).getEntry();
  
  private GenericEntry SBArmPIDP = extensionarmTab.add("PID P", 0.04).withPosition(0, 1).getEntry();
  private GenericEntry SBArmPIDI = extensionarmTab.add("PID I", 0.000000125).withPosition(1, 1).getEntry();
  private GenericEntry SBArmPIDD = extensionarmTab.add("PID D", 0.005).withPosition(2, 1).getEntry();

  private GenericEntry SBArmTargettedPosition = extensionarmTab.add("Arm Targetted Position", 0).withPosition(0, 2).getEntry();
  private GenericEntry SBArmTargettedPowerSpeed = extensionarmTab.add("Arm Targetted Power Speed", 0.15).withPosition(1, 2).getEntry();
  private GenericEntry SBArmHeldPID = extensionarmTab.add("Hold PID?",0).withPosition(0, 3).getEntry();
  private GenericEntry SBResetArmPosition = extensionarmTab.add("Reset Position?",0).withPosition(1,3).getEntry();
  private GenericEntry SBCurrentArmSetPoint = extensionarmTab.add("Current Arm Setpoint",0).withPosition(0, 4).getEntry();
  private GenericEntry SBArmSetPointEnable = extensionarmTab.add("Enable Arm Set Point",0).withPosition(2, 3).getEntry();
  
  

  /** Creates a new ExtensionArms. */
  public ExtensionArms() {
    m_extensionarm = new CANSparkMax(RobotMap.ExtensionArmConstants.armMotorID, MotorType.kBrushless);
    m_extensionarm.restoreFactoryDefaults();
    m_extensionarm.getEncoder().setPosition(0);
    m_extensionarm.setInverted(false);
    m_extensionarm.setIdleMode(IdleMode.kBrake);
    targettedPowerVelocity = 0.15;
    targettedPosition = 0;
    targettedSetPoint = 1;
  }

  public void runArmPower(double Percentage){
    m_extensionarm.set(Percentage);
  }

  public void shuffleSetPoint(int point){
    targettedSetPoint = point;
  }

  public void toggleArmSetpoint(int i){
    SBArmSetPointEnable.setInteger(i);
  }

  public void toggleArmPosition(int i){
    SBArmHeldPID.setInteger(i);
  }

  public void setTargettedPosition(double pos){
    SBArmTargettedPosition.setDouble(pos);
    targettedPosition = pos;
  }

  public double getPoint(){
    return targettedSetPoint; 
  }

  public void runSetPoint(){
    if (targettedSetPoint ==0){
      SBArmTargettedPosition.setDouble(1);
      targettedPosition = 1;
    }

    else if (targettedSetPoint ==1){
      SBArmTargettedPosition.setDouble(-95);
      targettedPosition = -95;

      //ORIGINAL -95
    }

    else if (targettedSetPoint == 4){
      SBArmTargettedPosition.setDouble(6);
      targettedPosition = 6;
      
    }
    else if (targettedSetPoint == 5){
      SBArmTargettedPosition.setDouble(-18);
      targettedPosition = -18;
      
    }
    

/* 
    if((targettedSetPoint ==0) && (m_extensionarm.getEncoder().getPosition()>=-0.25)){
      m_extensionarm.getEncoder().setPosition(0);

    }
*/
    m_extensionarm.getPIDController().setReference(targettedPosition, ControlType.kPosition);

    
 

  }


  public void runArmPosition(double Position){
    m_extensionarm.getPIDController().setReference(Position, ControlType.kPosition);
  }

  public void resetExtensionArmTarget(){
    SBArmTargettedPosition.setDouble(m_extensionarm.getEncoder().getPosition());
  }

 public double getPosition(){
  return m_extensionarm.getEncoder().getPosition();
 } 
 public double getVelocity(){
  return m_extensionarm.getEncoder().getVelocity();
 }

 public void setPID(double p, double i, double d){
  m_extensionarm.getPIDController().setP(p);
  m_extensionarm.getPIDController().setI(i);
  m_extensionarm.getPIDController().setD(d);
 }

 public double getTargettedPowerVelocity(){
  return targettedPowerVelocity;
 }

 public double getTargettedPosition(){
  return targettedPosition;
 }

 public void resetTargettedPosition(){
  targettedPosition = m_extensionarm.getEncoder().getPosition();
  SBArmTargettedPosition.setDouble(m_extensionarm.getEncoder().getPosition());
 }






  @Override
  public void periodic() {
    
    if (SBResetArmPosition.getDouble(0) ==1){
      m_extensionarm.getEncoder().setPosition(0);
    }
    if ((SBArmPIDD.getDouble(0)!= m_extensionarm.getPIDController().getP()) || (SBArmPIDI.getDouble(0)!= m_extensionarm.getPIDController().getI())||(SBArmPIDD.getDouble(0)!= m_extensionarm.getPIDController().getD(0))){
      setPID(SBArmPIDP.getDouble(0), SBArmPIDI.getDouble(0), SBArmPIDD.getDouble(0));
    }

    if (targettedPowerVelocity != SBArmTargettedPowerSpeed.getDouble(0)){
      targettedPowerVelocity = SBArmTargettedPowerSpeed.getDouble(0);
    }

    if(targettedPosition != SBArmTargettedPosition.getDouble(0)){
      targettedPosition = SBArmTargettedPosition.getDouble(0);
    }

    SBArmCurrentPosition.setDouble(m_extensionarm.getEncoder().getPosition());
    SBArmCurrentSpeed.setDouble(m_extensionarm.getEncoder().getVelocity());
    SBCurrentArmSetPoint.setDouble(getPoint());

    
    if (m_extensionarm.getEncoder().getPosition() >= 1){

      m_extensionarm.getEncoder().setPosition(0);
      targettedSetPoint = 0;
      setTargettedPosition(0);
   

    }

    if(SBArmHeldPID.getInteger(0) == 1){
      runArmPosition(targettedPosition);
    }

    if(SBArmSetPointEnable.getInteger(0) == 1){
      runSetPoint();
    }

    if (m_extensionarm.getEncoder().getPosition() >= -0.5){
      resetExtensionArmTarget();

    }
     // This method will be called once per scheduler run
  }
    // This method will be called once per scheduler run
  
}