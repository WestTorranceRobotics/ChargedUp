// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class NonDifferentialArcadeDrive extends CommandBase {

  double xSpeed;
  double ySpeed;
  /** Creates a new NonDifferentialArcadeDrive. */
  public NonDifferentialArcadeDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // private void arcadeDrive(){
  //   xSpeed = MathUtil.applyDeadband(xSpeed, m_deadband);
  //   zRotation = MathUtil.applyDeadband(zRotation, m_deadband);

  //   xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
  //   zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

  //   // Square the inputs (while preserving the sign) to increase fine control
  //   // while permitting full power.
  //   if (squareInputs) {
  //     xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
  //     zRotation = Math.copySign(zRotation * zRotation, zRotation);
  //   }

  //   double leftSpeed = xSpeed - zRotation;
  //   double rightSpeed = xSpeed + zRotation;

  //   // Find the maximum possible value of (throttle + turn) along the vector
  //   // that the joystick is pointing, then desaturate the wheel speeds
  //   double greaterInput = Math.max(Math.abs(xSpeed), Math.abs(zRotation));
  //   double lesserInput = Math.min(Math.abs(xSpeed), Math.abs(zRotation));
  //   if (greaterInput == 0.0) {
  //     return new WheelSpeeds(0.0, 0.0);
  //   }
  //   double saturatedInput = (greaterInput + lesserInput) / greaterInput;
  //   leftSpeed /= saturatedInput;
  //   rightSpeed /= saturatedInput;

  //   m_leftMotor.set(speeds.left * m_maxOutput);
  //   m_rightMotor.set(speeds.right * m_maxOutput);
  // }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
