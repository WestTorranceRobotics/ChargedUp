// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionArms;
public class RunExtensionArmPower extends CommandBase {
  /** Creates a new RunExtensionArmPower. */
 ExtensionArms extensionArmSubsystem;
  public RunExtensionArmPower(ExtensionArms extensionarm) {
    this.extensionArmSubsystem = extensionarm;
    addRequirements(extensionarm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    extensionArmSubsystem.runArmPower(extensionArmSubsystem.getTargettedPowerVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    extensionArmSubsystem.runArmPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
