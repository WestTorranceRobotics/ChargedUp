// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLight;

public class SeeLimeLight extends CommandBase {
  LimeLight limeLight;

  /** Creates a new SeeLimeLight. */
  public SeeLimeLight(LimeLight limeLight) {
    this.limeLight = limeLight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limeLight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Checking for limelight");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return limeLight.getTV() == 1;
  }
}
