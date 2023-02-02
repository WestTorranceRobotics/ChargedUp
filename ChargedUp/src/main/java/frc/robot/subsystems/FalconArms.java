// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class FalconArms extends SubsystemBase {
  /** Creates a new FalconArms. */
  private WPI_TalonSRX leftArmMotor;
  private WPI_TalonSRX rightArmMotor;
  public FalconArms() {
    leftArmMotor = new WPI_TalonSRX(RobotMap.ArmMap.leftArmMotorCANID);
    rightArmMotor = new WPI_TalonSRX(RobotMap.ArmMap.rightArmMotorCANID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
