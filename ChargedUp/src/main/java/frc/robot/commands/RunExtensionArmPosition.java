// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionArms;


public class RunExtensionArmPosition extends CommandBase {
  ExtensionArms extensionArmSubsystem;
  XboxController controller;
  
  /** Creates a new RunExtensionArmPosition. */
  public RunExtensionArmPosition(ExtensionArms extensionArms, XboxController xbox) {
    this.controller = xbox;
    this.extensionArmSubsystem = extensionArms;
    addRequirements(extensionArms);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currenttargetposition = extensionArmSubsystem.getTargettedPosition();

    if (controller.getLeftY() >= 0.1){
      if (currenttargetposition +(controller.getRightY()*5) <= 300){
      currenttargetposition += (controller.getRightY()*5);
      extensionArmSubsystem.toggleArmSetpoint(0);
      extensionArmSubsystem.setTargettedPosition(currenttargetposition);
      extensionArmSubsystem.toggleArmPosition(1);
      }
      
    }

    else if (controller.getLeftY() <= -0.1){
      if (currenttargetposition -(controller.getRightY()*5) >= -300){
        currenttargetposition -= (controller.getRightY()*5);
        extensionArmSubsystem.toggleArmSetpoint(0);
        extensionArmSubsystem.setTargettedPosition(currenttargetposition);
        extensionArmSubsystem.toggleArmPosition(1);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
