// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveDistance extends CommandBase {
  DriveTrain dt;
  double distance;
  boolean isFinished;
  /** Creates a new DriveDistance. */
  public DriveDistance(DriveTrain dt, double distance/*,Intake intake,boolean runintake*/) {
    this.dt = dt;
    this.distance = distance;
    this.isFinished = false;

    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
    // dt.distancePIDDrive(distance);
    if (distance >=0){
    dt.TankDrive(0.6 * Math.signum(distance), 0.6 * Math.signum(distance));
    }
    else{
    dt.TankDrive(0.5 * Math.signum(distance), 0.5 * Math.signum(distance));

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.TankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
