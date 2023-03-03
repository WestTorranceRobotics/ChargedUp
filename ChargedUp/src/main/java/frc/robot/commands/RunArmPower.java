// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Arms;


import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunArmPower extends CommandBase {
  /** Creates a new RunArmPower. */
  Arms armsubsystem;
  public RunArmPower(Arms armss) {
    this.armsubsystem = armss;
    addRequirements(armss);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armsubsystem.runArmPower(armsubsystem.getTargettedPowerVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armsubsystem.runArmPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
