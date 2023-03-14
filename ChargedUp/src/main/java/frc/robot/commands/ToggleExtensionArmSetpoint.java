// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.ExtensionArms;
import frc.robot.subsystems.Intake;

public class ToggleExtensionArmSetpoint extends CommandBase {
  /** Creates a new ToggleExtensionArmSetpoint. */
  ExtensionArms extensionarmsubsystem;
  Arms armSubsystem;
  int setpoint;
  boolean isFinished;
  public ToggleExtensionArmSetpoint(ExtensionArms extensionArms,Arms arms, int point) {
    this.setpoint = point;
    this.armSubsystem = arms;
    this.extensionarmsubsystem = extensionArms;
    isFinished = false;
    addRequirements(extensionArms,arms);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    extensionarmsubsystem.toggleArmPosition(0);

    if (setpoint ==1){
      if((armSubsystem.getTargettedSetPoint() != 0)&& (armSubsystem.getPosition() <= -10)){
      extensionarmsubsystem.shuffleSetPoint(setpoint);
      extensionarmsubsystem.toggleArmSetpoint(1);
      isFinished = true;
      }

      isFinished = true;

    }
    else{
      extensionarmsubsystem.shuffleSetPoint(setpoint);
      extensionarmsubsystem.toggleArmSetpoint(1);
      isFinished = true;
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
