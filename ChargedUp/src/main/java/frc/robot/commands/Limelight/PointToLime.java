// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;

public class PointToLime extends CommandBase {

DriveTrain driveTrain;
LimeLight limeLight;
private double tolerence = 0.05;

  /** Creates a new PointToLime. */
  public PointToLime(DriveTrain driveTrain,LimeLight limeLight) {

this.driveTrain = driveTrain;
this.limeLight = limeLight;
addRequirements(driveTrain,limeLight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double calcuation = MathUtil.clamp(limeLight.pidCaluculation(), -1, 1);
    System.out.println(calcuation);
    driveTrain.TankDrive(-calcuation, calcuation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.TankDrive(0, 0);
    limeLight.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return limeLight.getIsFinished();
    return Math.abs(limeLight.getTX()) < tolerence || limeLight.getTV() == 0;
  }
}