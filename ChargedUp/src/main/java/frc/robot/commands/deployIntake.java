// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class deployIntake extends CommandBase {
  Intake intake;
  double angle;
  boolean isFinished = false;
  public deployIntake(Intake intake, double angle) {
    this.intake = intake;
    this.angle = angle;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.resetIntakeDistance();
    intake.setDeployMotorSpeeds(0.5);
    if (angle <0) {
      intake.setDeployMotorSpeeds(-0.5);
    }
    
  }

  @Override
  public void execute() {
    if (angle>0){
    if (intake.getIntakeDistance() >= angle) {
      this.isFinished = true;
    }
  }
    if (angle <0) {
      if (intake.getIntakeDistance() <= angle) {
        this.isFinished = true;
      }
    }
    else {
      this.isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setDeployMotorSpeeds(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}
