// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.HelperCommands;

import java.util.function.DoubleBinaryOperator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms;

public class EndWhenRotatedToPoint extends CommandBase {
  double target;
  double tolerence;
  Arms arms;
  /** Creates a new EndWhenRotatedToPoint. */
  public EndWhenRotatedToPoint(Arms arms,double target ) {
    this.arms = arms;
    this.target = target;
    tolerence = 2;
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
    if (target >= arms.getPosition())
      if (arms.getPosition() >= target){
        return true;
      }
    else if (target <= arms.getPosition())
      if (arms.getPosition() <= target){
        return true;
      }
    return false;
  }
}
