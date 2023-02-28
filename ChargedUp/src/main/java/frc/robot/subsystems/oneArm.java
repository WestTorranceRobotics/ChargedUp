// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class oneArm extends SubsystemBase {
  CANSparkMax leftMotor;
  CANSparkMax rightMotor;
  Encoder armEncoder;
  
  /** Creates a new oneArm. */
  public oneArm() {
    leftMotor = new CANSparkMax(100, MotorType.kBrushless);
    rightMotor = new CANSparkMax(38, MotorType.kBrushless);

    leftMotor.setInverted(true);

    leftMotor.follow(rightMotor);

    armEncoder = new Encoder(100, 38);
    armEncoder.setDistancePerPulse(1/10);

  }

  public CANSparkMax getLeftMotor() {
    return leftMotor;
  }

  public CANSparkMax getRightMotor() {
    return rightMotor;
  }

  public void setMotor(double speed) {
    rightMotor.set(speed);
  }

  public Encoder getEncoder() {
    return armEncoder;
  }

  public double getEncoderDistance() {
    return armEncoder.getDistancePerPulse();
  }

  public void resetEncoder() {
    armEncoder.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
