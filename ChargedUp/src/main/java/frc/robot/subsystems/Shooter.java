// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.networktables.GenericEntry;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax shootMotorFollower;
  private CANSparkMax shootMotorLeader;
  private ShuffleboardTab shootTab = Shuffleboard.getTab("ShootTab");
  private GenericEntry SBshooterLeaderVelocity = shootTab.add("Leader Velocity", 0).withPosition(0, 0).getEntry();
  private GenericEntry SBshooterFollowerVelocity = shootTab.add("Follower Velocity", 0).withPosition(1, 0).getEntry();
  private GenericEntry SBPIDP = shootTab.add("PID P Value", 0).withPosition(0, 1).getEntry();
  private GenericEntry SBPIDI = shootTab.add("PID I Value", 0).withPosition(1, 1).getEntry();
  private GenericEntry SBPIDD = shootTab.add("PID D Value", 0).withPosition(2, 1).getEntry();
  private GenericEntry SBTargetVelocity =  shootTab.add("Target Velocity", 0).withPosition(0, 2).getEntry();
  private GenericEntry SBTargetPosition =  shootTab.add("Target Position", 0).withPosition(1, 2).getEntry();
  private GenericEntry SBVelocityOrPosition = shootTab.add("Velocity/Position", true).withWidget(BuiltInWidgets.kBooleanBox).withPosition(1, 2).getEntry();

  
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

  public void setPID(double p, double i, double d){
    shootMotorLeader.getPIDController().setP(p);
    shootMotorLeader.getPIDController().setI(i);
    shootMotorLeader.getPIDController().setD(d);

    shootMotorFollower.getPIDController().setP(p);
    shootMotorFollower.getPIDController().setI(i);
    shootMotorFollower.getPIDController().setD(d);





  }

  public void setVelocity (double velocity){
    shootMotorFollower.getPIDController().setReference(velocity,ControlType.kVelocity);

  
  }

  
  public void setPosition (double position){
    shootMotorFollower.getPIDController().setReference(position,ControlType.kPosition);

  }


  @Override
  public void periodic() {
    SBshooterFollowerVelocity.setDouble(shootMotorFollower.getEncoder().getVelocity());
    SBshooterLeaderVelocity.getDouble(shootMotorLeader.getEncoder().getVelocity());

    if ((shootMotorLeader.getPIDController().getP() != SBPIDP.getDouble(0)) || (shootMotorLeader.getPIDController().getI() != SBPIDI.getDouble(0) || (shootMotorLeader.getPIDController().getD() != SBPIDD.getDouble(0)))) {
      
    
      setPID(SBPIDP.getDouble(0), SBPIDI.getDouble(0), SBPIDD.getDouble(0));

    }
    // This method will be called once per scheduler run
  }
}
