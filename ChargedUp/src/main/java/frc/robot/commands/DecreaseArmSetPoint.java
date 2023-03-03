// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Arms;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DecreaseArmSetPoint extends CommandBase {
  /** Creates a new DecreaseArmSetPoint. */
  Arms armsubsystem;
  boolean isFinished =false;
  public DecreaseArmSetPoint(Arms arms) {
    armsubsystem = arms;
    addRequirements(arms);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armsubsystem.shuffleSetPoint(false);
    isFinished = true;
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
