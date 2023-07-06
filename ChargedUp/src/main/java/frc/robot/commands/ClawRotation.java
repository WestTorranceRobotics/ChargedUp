// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.LEDs.LEDs;

public class ClawRotation extends CommandBase {
  Claw clawsubsystem;
  XboxController controller;
  Arms arms;
  int dir = -1;
  LEDs leds;

  /** Creates a new ClawRotation. */
  public ClawRotation(Claw claw,XboxController xbox, Arms arms, LEDs leds) {
    this.clawsubsystem = claw;
    this.controller = xbox;
    this.arms = arms;
    this.leds = leds;

    addRequirements(claw);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  double[] hueRange = {60, 150};
  @Override
  public void execute() {
    if(!clawsubsystem.GetIsAuto()){
      if(arms.getTargettedSetPoint() == 1)
      {
        if (Math.abs(clawsubsystem.getIntakeVelocity()) < 600)
        {
          clawsubsystem.runClaw(-0.08);
          flashColor(80);
        }
        else
        {
          clawsubsystem.runClaw(-0.2);
          
          leds.SetModeNoise(hueRange);
        }
      }

      else
      {
        clawsubsystem.runClaw(-0.08);
        if (Math.abs(clawsubsystem.getIntakeVelocity()) < 200)
        {
          leds.SetModeSolid(80);
        }
        else
        {
          leds.SetModeNoise(hueRange);
        }
      }
    }

    // System.out.println(clawsubsystem.getIntakeVelocity());
    double leftTrigger =controller.getRawAxis(2);
    double rightTrigger = controller.getRawAxis(3);

    if(leftTrigger >= 0.5 || dir == 1){
      clawsubsystem.clockFlip();
      dir = 1;
    }
   

    if (rightTrigger >= 0.5 || dir == -1){
      clawsubsystem.counterClockFlip();
      dir = -1;

    }
  }

  private int counter = 0;
  private void flashColor(int hue){
    if(counter < 5){
      leds.SetModeSolid(hue);
    }
    else{
      leds.SetLEDsValue(25);
    }
    counter++;
    if(counter >= 10){
      counter = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clawsubsystem.stopRotate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
