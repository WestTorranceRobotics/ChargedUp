// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveTrain;

public class DriveToPitch extends CommandBase {

JoystickButton leftTrigger;
DriveTrain driveTrain;

  /** Creates a new DriveToYaw. */
  public DriveToPitch(JoystickButton leftTrigger, DriveTrain driveTrain) {

this.leftTrigger = leftTrigger;
this.driveTrain = driveTrain;

addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

double currentPitch = driveTrain.getPitch();

if (currentPitch > 0){

driveTrain.tankDrive(-0.1, -0.1);

}

if (currentPitch < 0){

driveTrain.tankDrive(0.1, 0.1);

}

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
