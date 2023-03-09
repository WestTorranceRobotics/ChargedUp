// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Joystick;

public class TankDrive extends CommandBase {
  /** Creates a new TankDrive. */
  DriveTrain drivetrain;

  private Joystick leftjoystick;
  private Joystick rightjoystick;
  public TankDrive(Joystick leftJoystick, Joystick rightJoystick, DriveTrain dt) {
    this.drivetrain = dt;
    this.leftjoystick = leftJoystick;
    this.rightjoystick = rightJoystick;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    drivetrain.TankDrive(leftjoystick.getY(),rightjoystick.getY());



    int x = 0;

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
