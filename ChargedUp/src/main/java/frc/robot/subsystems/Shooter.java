// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import frc.robot.RobotMap;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax shootMotorFollower;
  private CANSparkMax shootMotorLeader;
  
  public Shooter() {
    shootMotorFollower = new CANSparkMax(RobotMap.ShooterConstants.shooterFollower_ID, MotorType.kBrushless);
    shootMotorLeader = new CANSparkMax(RobotMap.ShooterConstants.shooterLeader_ID, MotorType.kBrushless);
    //leader
    shootMotorLeader.restoreFactoryDefaults();
    shootMotorLeader.setIdleMode(IdleMode.kCoast);
    //follower
    shootMotorFollower.restoreFactoryDefaults();
    shootMotorFollower.setIdleMode(IdleMode.kCoast);
    shootMotorFollower.follow(shootMotorLeader, true);
  }

  public void setPower(double power) {
    shootMotorLeader.set(power);
    shootMotorFollower.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
