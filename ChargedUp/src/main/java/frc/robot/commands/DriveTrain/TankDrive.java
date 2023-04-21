// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;


public class TankDrive extends CommandBase {
  DriveTrain drivetrain;
  Joystick lJoystick;
  Joystick rJoystick; 
  
  public TankDrive(Joystick lJoystick, Joystick rJoystick, DriveTrain dt) {
    this.drivetrain = dt;
    this.lJoystick = lJoystick;
    this.rJoystick = rJoystick;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double leftSpeed = MathUtil.applyDeadband(-lJoystick.getY(), 0.05);
    double rightSpeed = MathUtil.applyDeadband(-rJoystick.getY(), 0.05);
    drivetrain.TankDrive(leftSpeed,rightSpeed);

  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.StopDrive();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
