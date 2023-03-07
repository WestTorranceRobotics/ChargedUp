// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {
  private PIDController limePIDController;
  private boolean isFinished;


  /** Creates a new LimeLight. */
  public LimeLight() {
    limePIDController = new PIDController(0.1, 0, 0);
    limePIDController.setTolerance(2.0);
    isFinished = false;


  }

  public double getTX() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  public double pidCaluculation(){
    double calulation = MathUtil.clamp(limePIDController.calculate(getTX(), 0), -0.8, 0.8);

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
    // This method will be called once per scheduler run
  }
}
