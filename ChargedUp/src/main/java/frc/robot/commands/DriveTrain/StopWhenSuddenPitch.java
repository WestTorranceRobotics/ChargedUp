// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class StopWhenSuddenPitch extends CommandBase {
  DriveTrain dt;
  Timer timer = new Timer();
  double prevPitch;
  boolean isFinished = false;

  double velocityThreshold;

  /** Creates a new StopWhenPitchBack. */
  public StopWhenSuddenPitch(DriveTrain dt, double velocityThreshold) {
    this.dt = dt;
    this.velocityThreshold = velocityThreshold;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    prevPitch = dt.getPitch();
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitchVelocity = Math.abs(prevPitch - dt.getPitch())/timer.get();
    prevPitch = dt.getPitch();
    if(pitchVelocity >= velocityThreshold){
      isFinished = true;
    }

    timer.reset();
    dt.setSBPitchVelocity(pitchVelocity);
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
