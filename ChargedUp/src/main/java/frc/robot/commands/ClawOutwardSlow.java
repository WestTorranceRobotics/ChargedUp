// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
public class ClawOutwardSlow extends CommandBase {
  /** Creates a new ClawOutwardSlow. */
  Claw clawSubsystem;
  boolean isFinished;
  public ClawOutwardSlow(Claw claw) {
    this.clawSubsystem = claw;
    isFinished = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clawSubsystem.SetIsAuto(false);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    clawSubsystem.runClaw(0.5);
    //0.5

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clawSubsystem.runClaw(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
