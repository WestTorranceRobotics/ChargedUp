// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.HelperCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionArms;

public class EndWhenExtendedToPoint extends CommandBase {
  ExtensionArms extensionArms;
  double target;
  double tolerence;
  /** Creates a new EndWhenExtendedToPoint. */
  public EndWhenExtendedToPoint(ExtensionArms extensionArms, double target) {
    this.extensionArms = extensionArms;
    this.target = target;

    tolerence = 1;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(target - extensionArms.getPosition()) <= tolerence){
      return true;
    }
    return false;
  }
}
