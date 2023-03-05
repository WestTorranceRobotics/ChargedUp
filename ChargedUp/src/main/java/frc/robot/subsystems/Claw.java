// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//2 neos
//2 pnematics
// limit switches
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Claw extends SubsystemBase {

PneumaticsControlModule controlModule;

CANSparkMax motionMotor;
CANSparkMax powerMotor;

Solenoid leftSolenoid;
Solenoid rightSolenoid;
boolean direction;
DigitalInput upLimitSwitch;
DigitalInput downLimitSwitch;

  /** Creates a new Claw. */
  public Claw() {

PneumaticsControlModule controlModule = new PneumaticsControlModule(RobotMap.ClawMap.controlModuleCANID);
direction = true;
CANSparkMax motionMotor = new CANSparkMax(RobotMap.ClawMap.motionMotorCANID, MotorType.kBrushless);
CANSparkMax powerMotor = new CANSparkMax(RobotMap.ClawMap.powerMotorCANID, MotorType.kBrushless);

Solenoid leftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.ClawMap.leftSolenoidPort);
Solenoid rightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.ClawMap.rightSolenoidPort);

DigitalInput upLimitSwitch = new DigitalInput(RobotMap.ClawMap.upLimitSwitchChannel);
DigitalInput downLimitSwitch = new DigitalInput(RobotMap.ClawMap.downLimitSwitchChannel);

  }

public void runClaw(double power){

powerMotor.set(power);

}

public void extendClaw(boolean bol){

leftSolenoid.set(bol);
rightSolenoid.set(bol);

}




public boolean getUpSwitch(){

return upLimitSwitch.get();

}

public boolean getDownSwitch(){

return downLimitSwitch.get();

}

public void counterClockFlip(){


motionMotor.set(-RobotMap.ClawMap.motionMotorFlipPower);

if (getUpSwitch() == true){
  motionMotor.stopMotor();
}

}

public void clockFlip(){

motionMotor.set(RobotMap.ClawMap.motionMotorFlipPower);

if (getDownSwitch() == true){
  motionMotor.stopMotor();
}

}
}