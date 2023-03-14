// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.net.http.HttpResponse.PushPromiseHandler;

import com.fasterxml.jackson.databind.node.ArrayNode;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.Intake;

public class ToggleArmSetpoint extends CommandBase {
  boolean isFinished;
  Arms armsubsystem;
  Intake intakeSubsystem;
  int setpoint;
  /** Creates a new ToggleArmSetpoint. */
  public ToggleArmSetpoint(Arms arms,Intake intake, int point) {
    this.armsubsystem = arms;
    this.setpoint = point;
    this.intakeSubsystem = intake;
    isFinished = false;
    addRequirements(arms,intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (intakeSubsystem.getSolenoid()){
    armsubsystem.setSetPoint(setpoint);
    armsubsystem.toggleSetpoint(1);
    armsubsystem.togglePosition(0);
    
    isFinished = true;
    }

    else{
      if ((armsubsystem.getTargettedSetPoint()!=0)&&(armsubsystem.getPosition()>=-10)){
        if (setpoint !=0){
          armsubsystem.setSetPoint(setpoint);
          armsubsystem.toggleSetpoint(1);
          armsubsystem.togglePosition(0);
          isFinished = true;
        }
        
      }
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
