// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawRotation extends CommandBase {
  Claw clawsubsystem;
  XboxController controller;
  int dir = 0;
  /** Creates a new ClawRotation. */
  public ClawRotation(Claw claw,XboxController xbox) {
    this.clawsubsystem = claw;
    this.controller = xbox;
    addRequirements(claw);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(clawsubsystem.IsClosed())
    {
      clawsubsystem.runClaw(-0.08);
    }
    else
    {
      clawsubsystem.runClaw(-0.04);
    }


    double leftTrigger =controller.getLeftTriggerAxis();
    double rightTrigger = controller.getRightTriggerAxis();

    if(leftTrigger >= 0.5){
      clawsubsystem.clockFlip();
      dir = 1;
    }
   

    if (rightTrigger >= 0.5){
      clawsubsystem.counterClockFlip();
      dir = -1;

    }

    if(dir == 1 && clawsubsystem.getDownSwitch()){
      clawsubsystem.stopRotate();
    }
    if(dir == -1 && clawsubsystem.getUpSwitch()){
      clawsubsystem.stopRotate();
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
