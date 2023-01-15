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
  public TankDrive(Joystick leftjoystick, Joystick righJoystick, DriveTrain dt) {
    this.drivetrain = dt;
    this.leftjoystick = leftjoystick;
    this.rightjoystick = righJoystick;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.TankDrive(leftjoystick.getY(),rightjoystick.getY());
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
