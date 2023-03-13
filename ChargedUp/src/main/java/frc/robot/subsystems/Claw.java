// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//2 neos
//2 pnematics
// limit switches
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.networktables.GenericEntry;


import frc.robot.Robot;
import frc.robot.RobotMap;

public class Claw extends SubsystemBase {

PneumaticsControlModule controlModule;

CANSparkMax clawRotateMotor;
CANSparkMax clawIntakeMotor;

Solenoid leftSolenoid;
Solenoid rightSolenoid;
boolean direction;
DigitalInput upLimitSwitch;
DigitalInput downLimitSwitch;

private ShuffleboardTab clawTab = Shuffleboard.getTab("ClawTab");
private GenericEntry SBLeftLimit = clawTab.add("Left limit switch", false).withPosition(0, 0).getEntry();
private GenericEntry SBRightLimit = clawTab.add("Right limit switch", false).withPosition(1, 0).getEntry();
private GenericEntry SBClawPosition = clawTab.add("Claw Position", 0).withPosition(2, 0).getEntry();
private GenericEntry SBClawSpeed = clawTab.add("Claw Speed", 0).withPosition(3, 0).getEntry();
private GenericEntry SBClawTargetSpeed = clawTab.add("Claw Target Speed", 0).withPosition(0, 1).getEntry();
private GenericEntry SBClawSolenoid = clawTab.add("Claw Solenoid", false).withPosition(1, 1).getEntry();



  /** Creates a new Claw. */
  public Claw() {
    PneumaticsControlModule controlModule = new PneumaticsControlModule(RobotMap.ClawMap.controlModuleCANID);

    // controlModule.enableCompressorAnalog(110, 120);
    direction = true;
    clawRotateMotor = new CANSparkMax(RobotMap.ClawMap.motionMotorCANID, MotorType.kBrushless);
    clawRotateMotor.getEncoder().setPosition(0);
    clawIntakeMotor = new CANSparkMax(RobotMap.ClawMap.powerMotorCANID, MotorType.kBrushless);
    clawIntakeMotor.setIdleMode(IdleMode.kBrake);
    clawRotateMotor.setIdleMode(IdleMode.kBrake);
    leftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    //Solenoid rightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.ClawMap.rightSolenoidPort);

    upLimitSwitch = new DigitalInput(RobotMap.ClawMap.upLimitSwitchChannel);
    downLimitSwitch = new DigitalInput(RobotMap.ClawMap.downLimitSwitchChannel);

  }

  public void runClaw(double power){

  clawIntakeMotor.set(power);

  }



  public void extendClaw(boolean bol){

  leftSolenoid.set(bol);
  //rightSolenoid.set(bol);

  }

  public boolean getDirection(){
    return direction;
  }

  public void ToggleDirection(){
    direction = !direction;
  }


  public void rotateArm(double speed){
    clawRotateMotor.set(speed);

  }

  public void stopRotate(){
    clawRotateMotor.set(0);
  }

  public boolean getUpSwitch(){
    return !upLimitSwitch.get();
  }

  public boolean getDownSwitch(){

  return !downLimitSwitch.get();

  }

  public void counterClockFlip(){

    if ((getUpSwitch() == false) && (clawRotateMotor.getEncoder().getPosition() <= 45)){
      clawRotateMotor.set(0.15);

    }
    else
    {
      clawRotateMotor.set(0.0);
    }

  }

  public void clockFlip(){

    if (clawRotateMotor.getEncoder().getPosition()>=0){
      clawRotateMotor.set(-0.15);
    }
    else{
      clawRotateMotor.set(0.0);
    }
  }
  @Override
  public void periodic() {
    if(leftSolenoid.get())
    {
      runClaw(-0.08);
    }
    else
    {
      runClaw(-0.05);
    }


    SBClawSolenoid.setBoolean(leftSolenoid.get());
    SBLeftLimit.setBoolean(getDownSwitch());
    SBRightLimit.setBoolean(getUpSwitch());
    SBClawPosition.setDouble(clawRotateMotor.getEncoder().getPosition());
    SBClawSpeed.setDouble(clawRotateMotor.getEncoder().getVelocity());

  }
}