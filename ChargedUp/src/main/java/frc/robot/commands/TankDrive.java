// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;

public class TankDrive extends CommandBase {
  /** Creates a new TankDrive. */
  DriveTrain drivetrain;
  double zeroTolerence = 0.15;
  double maxTolerence = 0.9;
  double notMaxTopSpeed = 0.8;

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
    double leftSpeed = calculate(-leftjoystick.getY());
    double rightSpeed = calculate(-rightjoystick.getY());
    drivetrain.TankDrive(leftSpeed,rightSpeed);



  

  }

  double calculate(double speed){
    double abs = Math.abs(speed);
    if (abs < zeroTolerence){return 0;}
    return speed;
    // else if (abs < maxTolerence){return 1;}
    // else {
    //   double scaled = abs * notMaxTopSpeed / maxTolerence;
    //   return scaled * Math.signum(speed);
    // }
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
