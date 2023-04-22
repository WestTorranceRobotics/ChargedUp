// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ToggleWrist extends CommandBase {
  Claw claw;
  int dir;
  /** Creates a new ToggleClaw. */
  public ToggleWrist(Claw claw) {
    this.claw = claw;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(claw.getPosition()<=4){
      claw.clockFlip();
      dir = 1; 
    }
    else{
      claw.counterClockFlip();
      dir = -1;
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.stopRotate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double position = claw.getPosition();
    if(dir == 1){
      return (position >= 44);
    }
    if(dir == -1){
      return (position <= 1);
    }
    return false;
  }
}
