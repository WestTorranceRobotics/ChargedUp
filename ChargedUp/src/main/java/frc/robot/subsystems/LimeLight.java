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
  private GenericEntry topIsOpen = limelightTab.add("Top Spot",0).withPosition(1, 1).getEntry();
  private GenericEntry bottomIsOpen = limelightTab.add("Top Spot",0).withPosition(1, 2).getEntry();
  private GenericEntry topBound = limelightTab.add("Top Spot",0).withPosition(2, 1).getEntry();
  private GenericEntry bottomBound = limelightTab.add("Top Spot",0).withPosition(2, 2).getEntry();
  private double topRange = 0;
  private double bottomRange = 0;

  private PIDController limePIDController;
  private boolean isFinished;
  

  /** Creates a new LimeLight. */
  public LimeLight() {
    limePIDController = new PIDController(0.1, 0, 0);
    limePIDController.setTolerance(1.0);
    isFinished = false;


  }

  public double getTX() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  public void setPipeline(int pipline){

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").setInteger(pipline);

  }

  public double pidCaluculation(){


    double calulation = MathUtil.clamp(limePIDController.calculate(getTX(), 0), -0.7, 0.7);

    if(limePIDController.atSetpoint()){
      isFinished = true;
      return 0.0;
    }
    else{
      return Math.signum(calulation)*Math.max(calulation, 0.1);
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
    double boxHeight = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDouble(0);

    bottomBound.setDouble(getTX() - boxHeight/2);
    topBound.setDouble(getTX() + boxHeight/2);
    topIsOpen.setBoolean(getTX() + boxHeight/2 > topRange);
    bottomIsOpen.setBoolean(getTX() - boxHeight/2 < bottomRange);
    // This method will be called once per scheduler run
  }
}