// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

public class Intake extends SubsystemBase {
  WPI_TalonSRX intakeMotor1;
  WPI_TalonSRX intakeMotor2;
  Solenoid solenoid;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor1 = new WPI_TalonSRX(RobotMap.IntakeConstants.intakeMotor1_ID) ;
    intakeMotor2 = new WPI_TalonSRX(RobotMap.IntakeConstants.intakeMotor2_ID) ;
    solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

    intakeMotor1.setInverted(false);
    intakeMotor2.setInverted(false);



    
  }

  public void runIntake(boolean inverted){
    if (inverted){
    intakeMotor1.set(0.8);
    intakeMotor2.set(-0.8);
    }
    else{ 
    intakeMotor1.set(-0.8);
    intakeMotor2.set(0.8);
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
