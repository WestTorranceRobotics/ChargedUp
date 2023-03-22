// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;

public class DriveTrainDefualt extends CommandBase {
  DriveTrain drivetrain;
  private XboxController driverXboxController;

  LimeLight limeLight;

  double slowSpeed = 0.2;
  double fastSpeed = 1;
  private double tolerence = 0.5;

  /** Creates a new ArcadeDrive. */
  public DriveTrainDefualt(XboxController xboxController ,  DriveTrain dt, LimeLight limeLight) {
    this.drivetrain = dt;
    this.driverXboxController = xboxController;
    this.limeLight = limeLight;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, limeLight);
  }

  private void arcadeDrive ()
  {
    double speed = MathUtil.applyDeadband(-driverXboxController.getLeftY(), 0.05);
    double turn = MathUtil.applyDeadband(-driverXboxController.getRightX(), 0.05);

    drivetrain.arcadeDrive(speed,turn);
  }

  private void limelightAlign(){
    if(!(Math.abs(limeLight.getTX()) < tolerence || limeLight.getTV() == 0)){
      double calcuation = MathUtil.clamp(limeLight.pidCaluculation(), -1, 1);
      double calcScaled = calcuation * (fastSpeed - slowSpeed) + (slowSpeed * Math.signum(calcuation));
      drivetrain.TankDrive(-calcScaled, calcScaled);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(driverXboxController.getXButtonPressed()){
      limelightAlign();
    }
    else{
      arcadeDrive();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.StopDrive();
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
