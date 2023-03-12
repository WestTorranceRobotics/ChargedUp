// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.hal.util.BoundaryException;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class ControllerTankDrive extends CommandBase {
  /** Creates a new TankDrive. */
  DriveTrain drivetrain;

  private XboxController controller;

  public ControllerTankDrive(XboxController controller, DriveTrain dt) {
    this.drivetrain = dt;
    this.controller = controller;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = controller.getRawAxis(1);
    double rightSpeed = controller.getRawAxis(3);

    //new tankDrive System 
    double slowRange = 0.5;
    double threshold = 0.90;
    double maxSpeed = 0.6;

    if(Math.abs(leftSpeed) < threshold){ leftSpeed *= slowRange/threshold; }
    else {leftSpeed *= maxSpeed/Math.abs(leftSpeed);}

    if(Math.abs(rightSpeed) < threshold){ rightSpeed *= slowRange/threshold; }
    else {rightSpeed *= maxSpeed/Math.abs(rightSpeed);}
    //End Of new tankDrive System


    drivetrain.TankDrive(leftSpeed,rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.StopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
