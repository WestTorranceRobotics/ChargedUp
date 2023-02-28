// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import frc.robot.subsystems.oneArm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class turnArm extends CommandBase {
  oneArm oneArm;
  double angle;
 
  /** Creates a new turnArm. */
  public turnArm(oneArm oneArm, double angle) {
    this.oneArm = oneArm;
    this.angle = angle;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(oneArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
