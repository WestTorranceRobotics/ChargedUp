// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Spindexer;

public class CounterClockwiseSpin extends CommandBase {

JoystickButton operatorLB;

Spindexer spindexer;

  /** Creates a new CounterClockwiseSpin. */
  public CounterClockwiseSpin(JoystickButton operatorLB, Spindexer spindexer) {

this.operatorLB = operatorLB;
this.spindexer = spindexer;

addRequirements(spindexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

spindexer.spinLeft();

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
