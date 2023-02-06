// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

public class Intake extends SubsystemBase {
  CANSparkMax intakeMotor1;
  CANSparkMax intakeMotor2;
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor1 = new CANSparkMax(RobotMap.IntakeConstants.intakeMotor1_ID,MotorType.kBrushless);
    intakeMotor2 = new CANSparkMax(RobotMap.IntakeConstants.intakeMotor2_ID,MotorType.kBrushless);

    intakeMotor1.setIdleMode(IdleMode.kBrake);
    intakeMotor2.setIdleMode(IdleMode.kBrake);
    // intakeMotor1.setInverted(false);
    // intakeMotor2.setInverted(false);

    intakeMotor2.follow(intakeMotor1, true);


    
  }

  public void runIntake(boolean inverted){
    if (inverted){
    intakeMotor1.set(0.6);
    intakeMotor2.set(-0.6);
    }
    else{ 
    intakeMotor1.set(-0.6);
    intakeMotor2.set(0.6);
    }
 
  }

  public void stopIntake(){
    intakeMotor1.set(0);
    intakeMotor2.set(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
