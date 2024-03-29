// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveStraight extends CommandBase {
  DriveTrain dt;
  double speed;
  boolean isFinished = false;
  /** Creates a new DriveStraight. */
  public DriveStraight(DriveTrain dt, double speed){
    this.dt = dt;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    dt.TankDrive(speed, speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isFinished =true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.TankDrive(0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
