// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.RobotMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

public class Intake extends SubsystemBase {
  WPI_TalonSRX intakeMotor1;
  WPI_TalonSRX intakeMotor2;
  Solenoid leftsolenoid;
  Solenoid rightsolenoid;

  private ShuffleboardTab intakeTab =  Shuffleboard.getTab("Intake");
  private GenericEntry SBSolenoidDeployed = intakeTab.add("Solenoid",false).withPosition(0, 0).getEntry();
  private GenericEntry SBSpeedIntake =  intakeTab.add("Speed",1.0).withPosition(1, 0).getEntry();
  private GenericEntry SBSpeedIntake2 =  intakeTab.add("Speed 2",1.0).withPosition(2, 0).getEntry();



  //Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  /** Creates a new Intake. */
  public Intake() {

    //compressor.enableDigital();
    intakeMotor1 = new WPI_TalonSRX(RobotMap.IntakeConstants.intakeMotor1_ID) ;
    intakeMotor2 = new WPI_TalonSRX(RobotMap.IntakeConstants.intakeMotor2_ID) ;
    leftsolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    rightsolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 7);

    intakeMotor1.setNeutralMode(NeutralMode.Coast);
    intakeMotor2.setNeutralMode(NeutralMode.Coast);

    leftsolenoid.set(false);
    rightsolenoid.set(false);


    intakeMotor1.setInverted(false);
    intakeMotor2.setInverted(false);
    //intakeTab.addDouble("HIEEE", this::getAir); 
  }

  public void runIntake(boolean inverted){
    if (inverted){
    intakeMotor1.set(SBSpeedIntake.getDouble(0));
    intakeMotor2.set(-SBSpeedIntake2.getDouble(0));
    }
    else{ 
    intakeMotor1.set(-SBSpeedIntake.getDouble(0));
    intakeMotor2.set(SBSpeedIntake2.getDouble(0));
    }
 
  }

  public void stopIntake(){
    intakeMotor1.set(0);
    intakeMotor2.set(0);
  }

  public void toggleSolenoid(Boolean bol){
    leftsolenoid.set(bol);
    rightsolenoid.set(bol);
  }

  public boolean getSolenoid(){
    return leftsolenoid.get();
  }
  // public double getAir(){
  //   return compressor.getPressure();
  // }

  @Override
  public void periodic() {
    SBSolenoidDeployed.setBoolean(getSolenoid());

    // if(compressor.getPressure() >= 120){
    //   compressor.disable();
    // }

    
    // This method will be called once per scheduler run
  }
}
