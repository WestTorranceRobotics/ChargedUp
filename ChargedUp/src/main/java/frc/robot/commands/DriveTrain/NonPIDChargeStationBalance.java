// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class NonPIDChargeStationBalance extends CommandBase {
  DriveTrain dt;
  Timer timer = new Timer();

  boolean isEnded = false;
  boolean startedEndSeq = false;

  double startSpeed = 0.5;
  double reverseSpeed = -0.8;
  double reverseTime = 0.1;
  double velocityThresh = -2;

  double prevPitch;
  /** Creates a new PIDChargeStationBalance. */
  public NonPIDChargeStationBalance(DriveTrain dt) {
    this.dt = dt;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.TankDrive(startSpeed, startSpeed);
    prevPitch = dt.getPitch();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitchVelocity = prevPitch - dt.getPitch();
    prevPitch = dt.getPitch();
    System.out.println(pitchVelocity);
    if(pitchVelocity < velocityThresh){
      timer.reset();
      timer.start();
      dt.TankDrive(reverseSpeed, reverseSpeed);
      System.out.println("Ended");
    }
    if(startedEndSeq && timer.hasElapsed(reverseTime)){
      isEnded = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.TankDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isEnded;
  }
}
