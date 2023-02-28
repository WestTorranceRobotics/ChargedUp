// Copyright (c) First and other WPILib contributors.
/*  Open Source Software; you can modify and/or share it under the terms of
the WPILib BSD license file in the root directory of this project. */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class runIntakeForDistance extends CommandBase {
  Intake Intake;
  double speed;
  double angle;
  boolean end = false;
  double startingPos = Intake.getIntakeDistance(); 
  
  /** Creates a new runIntakeForDistance. */
  public runIntakeForDistance(Intake Intake, double speed, double angle) {
    this.Intake = Intake;
    this.speed = speed;
    this.angle = angle;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (angle < 0) {
      Intake.setDeployMotorSpeeds(-speed);
    } else {
      Intake.setDeployMotorSpeeds(speed);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (angle < 0) {
      if (Intake.getIntakeDistance() - startingPos <= angle)  {
       end = true;
      } 
    } else {
      if (Intake.getIntakeDistance() - startingPos >= angle)  {
        end = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.setDeployMotorSpeeds(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
