// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Test;

import javax.xml.namespace.QName;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveTrain;

public class FieldOrientedDrive extends CommandBase {
  /** Creates a new FieldOrientedDrive. */
  XboxController controller;
  DriveTrain dt;
  PIDController inPlacePID = new PIDController(0.015, 0, 0.001);
  PIDController movingPID = new PIDController(0.007, 0, 0.001);

  double wheelSeperation = 23;
  double maxRadius = 56;

  public FieldOrientedDrive(XboxController controller, DriveTrain dt) {
    this.controller = controller;
    this.dt = dt;

    inPlacePID.setTolerance(1);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double controllerAngle = controllerAngle(0,1);
    double targetAngle = dt.getYaw() - (-49.7);
    if (targetAngle > 180){ targetAngle -= 360; }
    if (targetAngle < -180){ targetAngle += 360; }
    
    double shortestAngle = shortestAngle(controllerAngle, targetAngle);

    double maxSpeed = 0.75;
    double minSpeed = 0.4;

    double power = -999;
    double speed = controller.getRawAxis(3) * 0.75;
    
    if(Math.abs(speed) < 0.01 && Math.abs(shortestAngle) < 200)
    {
      power = MathUtil.clamp(movingPID.calculate(shortestAngle, 0), -1, 1);
        boolean isNegative = power < 0;
        power *= maxSpeed - minSpeed;
        if (isNegative){power -= minSpeed;}
        else{power += minSpeed;}

        dt.TankDrive(power, -power);
    }
    else if(Math.abs(shortestAngle) < 2 || Math.abs(shortestAngle) > 200)
    {
      dt.TankDrive(speed, speed);
    }
    else if(Math.abs(shortestAngle) < 45)
    {
      double calc = MathUtil.clamp(inPlacePID.calculate(shortestAngle, 0), -1, 1);
      double radius;
      if(calc < 0)
      {
        radius = (-1-calc) * maxRadius;
      }
      else
      {
        radius = (1-calc) * maxRadius;
      }

      double leftRadius = radius - wheelSeperation/2;
      double rightRadius = radius + wheelSeperation/2;

      double leftSpeed = leftRadius/radius * speed;
      double rightSpeed = rightRadius/radius * speed;

      dt.TankDrive(leftSpeed, rightSpeed);      
    }
    else if(Math.abs(shortestAngle) < 200)
    {
        power = MathUtil.clamp(inPlacePID.calculate(shortestAngle, 0), -1, 1);
        boolean isNegative = power < 0;
        power *= maxSpeed - minSpeed;
        if (isNegative){power -= minSpeed;}
        else{power += minSpeed;}

        dt.TankDrive(power, -power);
    }
  }

  private double shortestAngle(double start, double target)
  {
    double angle = target-start;
    if (angle > 180) {
      angle -= 360;
    }
    if (angle <= -180) {
      angle += 360;
    }

    return angle;
  }

  private double controllerAngle(int x, int y){
    double yPos = -controller.getRawAxis(y);
    double xPos = -controller.getRawAxis(x);
    double radToDeg = 180/Math.PI;
    double angle;
    double tolerence = 0.01;
    double zeroTolerence = 0.1;

    if(Math.abs(xPos) < zeroTolerence && Math.abs(yPos) < zeroTolerence){
      angle = -999;
    }
    else if(Math.abs(xPos) < tolerence){
      if(yPos > 0){ angle = 0; }
      else { angle = 179.999; }
    }
    else
    {
      angle = Math.atan(xPos/yPos) * radToDeg;
      if(yPos < 0){
        if(xPos < 0){ angle -= 180; }
        else{ angle += 180; }
      }
    }

    return angle;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
