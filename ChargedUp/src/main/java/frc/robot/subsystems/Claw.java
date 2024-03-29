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

CANSparkMax motionMotor;
CANSparkMax intakeMotor;
boolean clawstate;
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


boolean isAuto = false;


  /** Creates a new Claw. */
  public Claw() {
    PneumaticsControlModule controlModule = new PneumaticsControlModule(RobotMap.ClawMap.controlModuleCANID);

    // controlModule.enableCompressorAnalog(110, 120);
direction = true;
clawstate = true;
motionMotor = new CANSparkMax(RobotMap.ClawMap.motionMotorCANID, MotorType.kBrushless);
motionMotor.getEncoder().setPosition(0);
intakeMotor = new CANSparkMax(RobotMap.ClawMap.powerMotorCANID, MotorType.kBrushless);
intakeMotor.setIdleMode(IdleMode.kBrake);
motionMotor.setIdleMode(IdleMode.kBrake);
leftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
//Solenoid rightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.ClawMap.rightSolenoidPort);

upLimitSwitch = new DigitalInput(RobotMap.ClawMap.upLimitSwitchChannel);
downLimitSwitch = new DigitalInput(RobotMap.ClawMap.downLimitSwitchChannel);
leftSolenoid.set(false);

  }


  public double getIntakeVelocity(){
    return intakeMotor.getEncoder().getVelocity();
  }

public void runClaw(double power){

intakeMotor.set(power);

}

public boolean GetIsAuto(){
  return isAuto;
}

public void SetIsAuto(boolean newVal)
{
  isAuto = newVal;
}

public void closeClaw(boolean bol){

leftSolenoid.set(bol);
//rightSolenoid.set(bol);

}



public void rotateArm(){
//  motionMotor.set(SBClawTargetSpeed.getDouble(0));

}

public void stopRotate(){
  motionMotor.set(0);
}

public boolean getUpSwitch(){

return !upLimitSwitch.get();

}

public boolean getDownSwitch(){

return !downLimitSwitch.get();

}

public void counterClockFlip(){
  SBLeftLimit.setBoolean(false);
  if ((motionMotor.getEncoder().getPosition()>= 0.1)){
    motionMotor.set(-0.15);
    clawstate = true;
    SBRightLimit.setBoolean(false);
  }
  else
  {
    SBRightLimit.setBoolean(true);
    motionMotor.set(-0.05);
  }

}

public void clockFlip(){
  SBRightLimit.setBoolean(false);
  if ((motionMotor.getEncoder().getPosition()<=2.21)){
    motionMotor.set(0.15);
    clawstate = false;
    SBLeftLimit.setBoolean(false);
  }

  else{
    motionMotor.set(0.05);
    SBLeftLimit.setBoolean(true);
  }
}
public double getPosition(){
  return motionMotor.getEncoder().getPosition();
}

public boolean IsClosed(){
  return leftSolenoid.get();
}
public boolean getClawState(){
  return clawstate;
}

@Override
public void periodic() {
  //powerMotor.set(-0.08);

  // SBClawSolenoid.setBoolean(leftSolenoid.get());
  SBClawSolenoid.setBoolean(motionMotor.getIdleMode() == IdleMode.kBrake);
  // SBLeftLimit.setBoolean(getDownSwitch());
  // SBRightLimit.setBoolean(getUpSwitch());
  SBClawPosition.setDouble(motionMotor.getEncoder().getPosition());
  SBClawSpeed.setDouble(motionMotor.getEncoder().getVelocity());
}
}