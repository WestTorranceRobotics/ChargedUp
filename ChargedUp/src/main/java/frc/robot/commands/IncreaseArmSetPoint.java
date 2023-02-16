// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms;

public class IncreaseArmSetPoint extends CommandBase {
  /** Creates a new IncreaseArmSetPoint. */

  boolean isFinished = false;
  Arms armsubsystem;
  public IncreaseArmSetPoint(Arms arms) {
    armsubsystem = arms;
    addRequirements(arms);
    isFinished = true;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armsubsystem.shuffleSetPoint(true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
