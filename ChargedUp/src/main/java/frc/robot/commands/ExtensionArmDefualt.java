// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ExtensionArms;

public class ExtensionArmDefualt extends CommandBase {
  ExtensionArms arm;
  XboxController controller;
  /** Creates a new RotateArmDefualt. */
  public ExtensionArmDefualt(ExtensionArms arm, XboxController controller) {
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
    if(controller.getPOV() == 315 || controller.getPOV() == 0 || controller.getPOV() == 45)
    {
      arm.runArmPower(arm.getTargettedPowerVelocity());
    }
    else if(controller.getPOV() == 135 || controller.getPOV() == 180 || controller.getPOV() == 225)
    {
      arm.runArmPower(-arm.getTargettedPowerVelocity());
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
