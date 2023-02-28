// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class arm extends SubsystemBase {
  Encoder Arm;
  CANSparkMax rightMotor;
  CANSparkMax leftMotor;

  public arm() {
    Arm = new Encoder(0,0);
    rightMotor = new CANSparkMax(1,MotorType.kBrushless);
    leftMotor = new CANSparkMax(2, MotorType.kBrushless);

    leftMotor.setInverted(true);
    leftMotor.follow(rightMotor);
    Arm.setDistancePerPulse(360/20);
    
  }
  public void setSpeed(double speed) {
    rightMotor.set(speed);
    leftMotor.set(speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
