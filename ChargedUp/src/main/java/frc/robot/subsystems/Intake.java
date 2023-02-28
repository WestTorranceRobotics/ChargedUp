// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax rightIntakeDeployer;
  private CANSparkMax leftIntakeDeployer;
  private CANSparkMax Roller;
  private Encoder IntakeEncoder = new Encoder(0, 1);

  public Intake() {
    rightIntakeDeployer = new CANSparkMax(3, MotorType.kBrushless);
    leftIntakeDeployer = new CANSparkMax(4, MotorType.kBrushless);
    Roller = new CANSparkMax(5, MotorType.kBrushless);

    leftIntakeDeployer.setInverted(true);
    rightIntakeDeployer.setInverted(false);
    
    leftIntakeDeployer.setIdleMode(IdleMode.kBrake);
    rightIntakeDeployer.setIdleMode(IdleMode.kBrake);
    Roller.setIdleMode(IdleMode.kCoast);

    leftIntakeDeployer.follow(rightIntakeDeployer);

    IntakeEncoder.setDistancePerPulse(Constants.intakeConstants.pulsesToDegreesRotate);


  }

  public void setDeployMotorSpeeds(double speed) {
    rightIntakeDeployer.set(speed);
  }

  public void setRollerMotorSpeed(double speed) {
    Roller.set(speed);
  }

  public void resetIntakeDistance() {
    IntakeEncoder.reset();
  }

  public double getIntakeDistance() {
    return IntakeEncoder.getDistance();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
