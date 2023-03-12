// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class RotateArmDefualt extends CommandBase {
  Arm arm;
  XboxController controller;
  /** Creates a new RotateArmDefualt. */
  public RotateArmDefualt(Arm arm, XboxController controller) {
    this.arm = arm;
    this.controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(controller.getLeftTriggerAxis() > 0.5)
    {
      arm.runArmPower(0.2);
    }
    else if(controller.getRightTriggerAxis() > 0.5){
      arm.runArmPower(-0.2);
    }
    else 
    {
      arm.runArmPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    arm.runArmPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
