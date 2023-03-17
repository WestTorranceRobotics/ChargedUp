// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Test;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;

public class TurnToDirection extends CommandBase {
  DriveTrain dt;
  PIDController PID;
  double targetAngle;
  double startYawOffset;
  double shortestAngle;

  double tolerence = 1;

  /** Creates a new TurnToDirection. */
  public TurnToDirection(DriveTrain dt, double target, double tolerence) {
    this.dt = dt;
    this.targetAngle = target;
    // this.startYawOffset = dt.getStartYawOffset();

    this.startYawOffset = -81.66;
    this.tolerence = tolerence;

    PID = dt.GetOrientationPID();
    PID.setTolerance(1);

    double currentAngle = dt.getYaw() - startYawOffset;
    if (currentAngle > 180){ currentAngle -= 360; }
    if (currentAngle < -180){ currentAngle += 360; }
    shortestAngle = shortestAngle(currentAngle, targetAngle);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = dt.getYaw() - startYawOffset;
    if (currentAngle > 180){ currentAngle -= 360; }
    if (currentAngle < -180){ currentAngle += 360; }
    shortestAngle = shortestAngle(currentAngle, targetAngle);
    System.out.println(shortestAngle);
    
    double calcuation = MathUtil.clamp(PID.calculate(shortestAngle), -1, 1);

    dt.TankDrive(calcuation, -calcuation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  private double shortestAngle(double start, double target)
  {
    double angle = target-start;
    if (angle > 180) {
      angle -= 360;
    }
    if (angle <= -180) {
      angle += 360;
    }

    return angle;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(shortestAngle) < tolerence;
  }
}
