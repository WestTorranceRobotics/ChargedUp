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
  boolean isFinished;
  double dir;
  /** Creates a new EndWhenRotatedToPoint. */
  public EndWhenRotatedToPoint(Arms arms,double target ) {
    this.arms = arms;
    this.target = target;
    this.isFinished = false;
    tolerence = 2;
    this.dir = 0;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (target >= arms.getPosition()){
      dir = 1;
    }
    else if (target <= arms.getPosition()){
      dir = -1;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  if (dir == 1){
    if (arms.getPosition() >= target){
      isFinished =true;
    }
  }
  else if (dir == -1){
    if (arms.getPosition() <= target){
      isFinished = true;
    }
  }
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
