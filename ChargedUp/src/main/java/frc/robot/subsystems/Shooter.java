// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;

import frc.robot.Robot;
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
  private GenericEntry SBPIDP = shootTab.add("PID P Value", RobotMap.ShooterConstants.kP).withPosition(0, 1).getEntry();
  private GenericEntry SBPIDI = shootTab.add("PID I Value", RobotMap.ShooterConstants.kI).withPosition(1, 1).getEntry();
  private GenericEntry SBPIDD = shootTab.add("PID D Value", RobotMap.ShooterConstants.kD).withPosition(2, 1).getEntry();
  private GenericEntry SBTargetVelocity =  shootTab.add("Target Velocity", 0).withPosition(0, 2).getEntry();
  private GenericEntry SBTargetPosition =  shootTab.add("Target Position", 0).withPosition(1, 2).getEntry();
  private GenericEntry SBVelocityOrPosition = shootTab.add("Velocity/Position", true).withWidget(BuiltInWidgets.kBooleanBox).withPosition(2, 2).getEntry();
  private GenericEntry SBshooterLeaderPosition = shootTab.add("Leader Position", 0).withPosition(0, 3).getEntry();
  private GenericEntry SBshooterFollowerPosition = shootTab.add("Follower Position", 0).withPosition(1, 3).getEntry();


  private double targetVelocity;
  private double targetPosition;
  private boolean velocityOrPosition;
  
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

    //Variables
    targetPosition = 0;
    targetVelocity = 0;
    velocityOrPosition = true;
  }

  public void setPower(double power) {
    shootMotorLeader.set(power);
  }

  public void setPID(double p, double i, double d){
    shootMotorLeader.getPIDController().setP(p);
    shootMotorLeader.getPIDController().setI(i);
    shootMotorLeader.getPIDController().setD(d);

    shootMotorFollower.getPIDController().setP(p);
    shootMotorFollower.getPIDController().setI(i);
    shootMotorFollower.getPIDController().setD(d);





  }

  public void resetPositon(){
    shootMotorFollower.getEncoder().setPosition(0);
    shootMotorLeader.getEncoder().setPosition(0);
  }

  public void setVelocity (double velocity){
    shootMotorLeader.getPIDController().setReference(velocity,ControlType.kVelocity);

  
  }

  
  public void setPosition (double position){
    shootMotorLeader.getPIDController().setReference(position,ControlType.kPosition);

  }

  public boolean getVeloctiyOrPosition(){
    return velocityOrPosition;
  }

  public double getTargetVelocity(){
    return targetVelocity;
  }

  public double getTargetPosition(){
    return targetPosition;
  }



  @Override
  public void periodic() {
    SBshooterFollowerVelocity.setDouble(shootMotorFollower.getEncoder().getVelocity());
    SBshooterLeaderVelocity.setDouble(shootMotorLeader.getEncoder().getVelocity());
    SBshooterLeaderPosition.setDouble(shootMotorLeader.getEncoder().getPosition());
    SBshooterFollowerPosition.setDouble(shootMotorFollower.getEncoder().getPosition());

    if ((shootMotorLeader.getPIDController().getP() != SBPIDP.getDouble(0)) || (shootMotorLeader.getPIDController().getI() != SBPIDI.getDouble(0) || (shootMotorLeader.getPIDController().getD() != SBPIDD.getDouble(0)))) {
      
      
      setPID(SBPIDP.getDouble(0), SBPIDI.getDouble(0), SBPIDD.getDouble(0));

    }

    if(targetPosition != SBTargetPosition.getDouble(0)){
      targetPosition = SBTargetPosition.getDouble(0);
    }

    if (targetVelocity != SBTargetVelocity.getDouble(0)){
      targetVelocity = SBTargetVelocity.getDouble(0);
    }

    if(velocityOrPosition != SBVelocityOrPosition.getBoolean(true)){
      velocityOrPosition = SBVelocityOrPosition.getBoolean(true);
    }
  



    // This method will be called once per scheduler run
  }

  
}
