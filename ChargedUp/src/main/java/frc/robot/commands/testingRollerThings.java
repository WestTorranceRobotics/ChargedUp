// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
  import frc.robot.subsystems.Intake;

public class testingRollerThings extends CommandBase {
  Intake Intake;
  double Speed;
  double Distance;
  boolean done = false;
  
  /** Creates a new testingRollerThings. */
  public testingRollerThings(Intake intake, double speed, double distance) {
    Intake = intake;
    Speed = speed;
    Distance = distance;
    addRequirements(Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Intake.rollerMotorSpeed(Speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.rollerMotorSpeed(0);

  }
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
