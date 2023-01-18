// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Claw extends SubsystemBase {

PneumaticsControlModule controlModule;

DoubleSolenoid solenoidOne;
DoubleSolenoid solenoidTwo;

  /** Creates a new Claw. */
  public Claw() {

PneumaticsControlModule controlModule = new PneumaticsControlModule(RobotMap.ClawMap.controlModuleCANID);

//DoubleSolenoid solenoidOne = new DoubleSolenoid(ModuleType.kRev, RobotMap.ClawMap.solenoidOneForwardChannel, RobotMap.ClawMap.solenoidOneReverseChannel);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
