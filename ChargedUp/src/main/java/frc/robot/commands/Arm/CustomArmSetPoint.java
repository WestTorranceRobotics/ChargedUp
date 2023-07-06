// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms;

public class CustomArmSetPoint extends CommandBase {
  Arms arms;
  double setpoint;

  boolean isFinished = false;
  /** Creates a new CustomArmSetPoint. */
  public CustomArmSetPoint(Arms arms, double setpoint) {
    this.arms = arms;
    this.setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arms);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arms.runCustomArmSetPoint(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isFinished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isFinished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
