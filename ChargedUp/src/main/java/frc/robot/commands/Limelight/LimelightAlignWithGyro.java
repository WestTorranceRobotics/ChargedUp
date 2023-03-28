// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveTrain.SeeLimeLight;
import frc.robot.commands.Test.TurnToDirection;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimelightAlignWithGyro extends SequentialCommandGroup {
  /** Creates a new LimelightAlignWithGyro. */
  public LimelightAlignWithGyro(DriveTrain dt, LimeLight limeLight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new SeeLimeLight(limeLight),
        new TurnToDirection(dt, 0, 3)
      ),
      new PointToLime(dt, limeLight),
      new PointToLime(dt, limeLight)
    );
  }
}
