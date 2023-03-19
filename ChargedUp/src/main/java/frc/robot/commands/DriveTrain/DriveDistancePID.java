// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveDistancePID extends CommandBase {
  /** Creates a new DriveDistancePID. */
  DriveTrain dt;
  double distance;
  boolean isFinished;
  public DriveDistancePID(DriveTrain driveTrain, double distance) {
    this.dt = driveTrain;
    this.distance = distance;
    this.isFinished = false;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dt.distancePIDDrive(distance);
    
    if (distance >= 0 ){
      if (dt.getLeftDistance() >= distance-10 ){
        isFinished = true;
      }
    }
    else if (distance < 0 ){
      if (dt.getLeftDistance() <= distance+50){
        isFinished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
