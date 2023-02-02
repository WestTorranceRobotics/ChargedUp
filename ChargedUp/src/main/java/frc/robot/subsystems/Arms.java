// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arms extends SubsystemBase {
  private CANSparkMax leftArmMotor;
  private CANSparkMax rightArmMotor;
  /** Creates a new Arms. */
  public Arms() {
    leftArmMotor = new CANSparkMax(RobotMap.ArmMap.leftArmMotorCANID, MotorType.kBrushless);
    rightArmMotor = new CANSparkMax(RobotMap.ArmMap.rightArmMotorCANID, MotorType.kBrushless);

    leftArmMotor.getPIDController().setP(1);
    leftArmMotor.getPIDController().setD(1);

    rightArmMotor.getPIDController().setP(1);
    rightArmMotor.getPIDController().setD(1);

    TalonFX x = new TalonFX(0);
    
  }

  public void ArmGrab(){
    leftArmMotor.set(0.5);
    rightArmMotor.set(0.5);
  }

  public void ArmUngrab(){
    leftArmMotor.set(-0.5);
    rightArmMotor.set(-0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
