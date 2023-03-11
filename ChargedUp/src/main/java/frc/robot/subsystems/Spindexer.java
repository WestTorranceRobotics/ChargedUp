// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Spindexer extends SubsystemBase {

  public CANSparkMax Spinner;
  //private RelativeEncoder encoder;

  /** Creates a new Spindexer. */
  public Spindexer() {
    Spinner = new CANSparkMax(RobotMap.SpindexerMap.spinnerCANID, MotorType.kBrushless);

    Spinner.setIdleMode(IdleMode.kBrake);
    Spinner.setInverted(RobotMap.SpindexerMap.clockwise);

    // encoder = Spinner.getEncoder();
    // encoder.setPositionConversionFactor(RobotMap.SpindexerMap.motorRevToRotation);
    // encoder.setInverted(RobotMap.SpindexerMap.clockwise);
  }
  
  public void spin(double power){
    Spinner.set(power);
  }

  public double GetPosition(){
    return Spinner.getEncoder().getPosition();
  }

  public void ResetPosition(){
    Spinner.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}