// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ClawTurning extends CommandBase {
  /** Creates a new ClawTurning. */
  Claw clawsubsystem;
  boolean spindirection;
  boolean isFinished;
  public ClawTurning(Claw claw,boolean direction) {
    this.clawsubsystem = claw;
    this.spindirection = direction;
    this.isFinished = false;
    addRequirements(clawsubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (spindirection){
      clawsubsystem.clockFlip();
      }
    else{
      clawsubsystem.counterClockFlip();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clawsubsystem.stopRotate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
