// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {
  private PIDController limePIDController;
    private boolean isFinished;
  private ShuffleboardTab limelightTab = Shuffleboard.getTab("limelighttab");
  private GenericEntry SBspeedout =  limelightTab.add("Speed out",0).withPosition(0, 0).getEntry();
  private double speedout;

  /** Creates a new LimeLight. */
  public LimeLight() {
    speedout = 0;
    limePIDController = new PIDController(1, 0, 0);
    limePIDController.setTolerance(1.0);
    isFinished = false;


  }


  public double getTX() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  public double pidCaluculation(){

    if (getTX() == 0){
      speedout = 100;
      return 0.5;

    }
    double calculation = MathUtil.clamp(limePIDController.calculate(getTX(), 0), -0.8, 0.8);
    speedout = calculation;
    if(limePIDController.atSetpoint()){
      isFinished = true;
    
      return 0.0;
    }
    else{
      return Math.signum(calculation)*Math.max(calculation, 0.1);
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
    SBspeedout.setDouble(speedout);
    // This method will be called once per scheduler run
  }
}
