// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;

public class RunArmPosition extends CommandBase {
  /** Creates a new RunArmPosition. */
  Arm armsubsystem;
  XboxController controller;
  public RunArmPosition(Arm armss,XboxController Xbox) {
    controller = Xbox;
    armsubsystem =armss;
    addRequirements(armss);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    double y = -controller.getLeftY();
    if (y >= 0.1){
      //armsubsystem.setTargttedPosition(armsubsystem.getPosition());
      double currenttargetposition = armsubsystem.getTargettedPosition();
      if (currenttargetposition + (y*0.15)<=0){
      currenttargetposition += (y*0.15);
      armsubsystem.toggleSetpoint(0);
      armsubsystem.setTargttedPosition(currenttargetposition);
      armsubsystem.togglePosition(1);
      }
      
    }

    else if (y <= -0.1){
      //armsubsystem.setTargttedPosition(armsubsystem.getPosition());
      double currenttargetposition = armsubsystem.getTargettedPosition();
      if (currenttargetposition +(y*0.15) >= -72){
        currenttargetposition += (y*0.15);
        armsubsystem.toggleSetpoint(0);
        armsubsystem.setTargttedPosition(currenttargetposition);
        armsubsystem.togglePosition(1);
      }
    }
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