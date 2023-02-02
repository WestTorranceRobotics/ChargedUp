// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterPIDController extends CommandBase {
  /** Creates a new ShooterPIDController. */
  Shooter shooterSubsystem;
  public ShooterPIDController(Shooter ss) {
    this.shooterSubsystem = ss;
    addRequirements(ss);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterSubsystem.getVeloctiyOrPosition()){
      shooterSubsystem.setVelocity(shooterSubsystem.getTargetVelocity());
    }
    else{
      shooterSubsystem.setPosition(shooterSubsystem.getTargetPosition());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setPower(0);;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
