// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {
  ShuffleboardTab limelightTab = Shuffleboard.getTab("LimeLight");
  private GenericEntry topIsOpen = limelightTab.add("Top Spot",false).withPosition(1, 1).getEntry();
  private GenericEntry bottomIsOpen = limelightTab.add("Bottom Spot",false).withPosition(1, 2).getEntry();
  private GenericEntry topBound = limelightTab.add("Top Bound",0).withPosition(2, 1).getEntry();
  private GenericEntry bottomBound = limelightTab.add("Bottom Bound",0).withPosition(2, 2).getEntry();
  private GenericEntry SBIsFinished =  limelightTab.add("Is Finished?",false).withPosition(0, 3).getEntry();
  private GenericEntry SBReturnSpeed =  limelightTab.add("Speed?",0).withPosition(1, 3).getEntry();
  private GenericEntry SBRotationKp = limelightTab.add("Rotation kP",0.002).withPosition(0, 4).getEntry();
  private GenericEntry SBRotationKi = limelightTab.add("Rotation kI",0).withPosition(1, 4).getEntry();
  private GenericEntry SBRotationKd = limelightTab.add("Rotation kD",0.0005).withPosition(2, 4).getEntry();
  
  private double middle = 22;

  private PIDController limePIDController;
  private boolean isFinished;
  double a_kp;
  double a_ki;
  double a_kd;
  

  /** Creates a new LimeLight. */
  public LimeLight() {
    limePIDController = new PIDController(0.1, 0, 0.05);
    a_kp = 0.1;
    a_ki = 0;
    a_kd = 0.05;
    limePIDController.setTolerance(2.5);
    isFinished = false;


  }

  public double getTX() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  public double getTY() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  public double getTV() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  }

  public void setPipeline(int pipline){

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").setInteger(pipline);

  }

  public double pidCaluculation(){


    double calculation = MathUtil.clamp(limePIDController.calculate(getTX(), 0), -0.7, 0.7);

    if(limePIDController.atSetpoint()){
      isFinished = true;
      return 0.0;
    }
    else{
      return Math.signum(calculation)*Math.max(Math.abs(calculation), 0.1);
    }

  }

  public void reset(){
    isFinished = false;
  }

  public boolean getIsFinished(){
    return isFinished;
  }
  

  @Override
  public void periodic() {
    SBIsFinished.setBoolean(isFinished);
    double calculation = MathUtil.clamp(limePIDController.calculate(getTX(), 0), -0.7, 0.7);
    SBReturnSpeed.setDouble( Math.signum(calculation)*Math.max(calculation, 0.1));
   
    double boxHeight = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDouble(0);
    double ty = getTY() * (320/40);
    bottomBound.setDouble(ty - boxHeight/2);
    topBound.setDouble(ty + boxHeight/2);
    topIsOpen.setBoolean(ty + boxHeight/2 > middle && getTV() == 1);
    bottomIsOpen.setBoolean(ty - boxHeight/2 < middle && getTV() == 1);
    // This method will be called once per scheduler run

    
    if ((a_kp != SBRotationKp.getDouble(0)) || (a_ki != SBRotationKi.getDouble(0)) || (a_kd != SBRotationKd.getDouble(0))){
      a_kp = SBRotationKp.getDouble(0);
      a_ki = SBRotationKi.getDouble(0);
      a_kd = SBRotationKd.getDouble(0);
      limePIDController.setPID(a_kp, a_ki, a_kd);
    }

  }
}