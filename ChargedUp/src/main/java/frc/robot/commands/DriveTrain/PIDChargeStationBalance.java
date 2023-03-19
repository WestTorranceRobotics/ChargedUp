// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveTrain;

public class PIDChargeStationBalance extends CommandBase {
  DriveTrain dt;
  double startPitch;
  PIDController gyroPID;

  /** Creates a new PIDChargeStationBalance. */
  public PIDChargeStationBalance(DriveTrain dt, double startPitch) {
    this.dt = dt;
    this.startPitch = startPitch;
    this.gyroPID = dt.getGyroPID();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gyroPID.setTolerance(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double maxSpeed = 0.4;
    double startPitchOffset = 0;

    double currentOrientation = dt.getPitch() - startPitchOffset;
    if (currentOrientation > 180){ currentOrientation -= 360; }
    if (currentOrientation < -180){ currentOrientation += 360; }

    double calculation = MathUtil.clamp(gyroPID.calculate(currentOrientation, 0), -maxSpeed, maxSpeed);
    dt.TankDrive(calculation, calculation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.TankDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
