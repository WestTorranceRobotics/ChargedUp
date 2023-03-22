// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ArcadeDrive extends CommandBase {
  DriveTrain drivetrain;
  

  private XboxController driverXboxController;
  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(XboxController xboxController ,  DriveTrain dt) {
    this.drivetrain = dt;
    this.driverXboxController = xboxController;
    addRequirements(drivetrain);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("left y :" + driverXboxController.getLeftY());
    // System.out.println("right x :" + driverXboxController.getRawAxis(2));
    double speed = MathUtil.applyDeadband(-driverXboxController.getLeftY(), 0.05);
    double turn = MathUtil.applyDeadband(-driverXboxController.getRightX(), 0.05);

    drivetrain.arcadeDrive(speed,turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.TankDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
