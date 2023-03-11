// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Spindexer;

public class SpindexerClockwise extends CommandBase {
  Spindexer spindexer;
  /** Creates a new SpindexerClockwise. */
  public SpindexerClockwise(Spindexer spindexer) {
    this.spindexer = spindexer;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spindexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    spindexer.ResetPosition();
    spindexer.spin(RobotMap.SpindexerMap.spindexerSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spindexer.spin(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(spindexer.GetPosition() >= 5){
      return true;
    }
    return false;
  }
}
