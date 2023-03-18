// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.CompleteAutos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ToggleIntakeSolenoid;
import frc.robot.commands.AutoCommands.HelperCommands.Delay;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BasicGrabSecondCube extends SequentialCommandGroup {
  DriveTrain dt;
  Intake intake;
  /** Creates a new BasicGrabSecondCube. */
  public BasicGrabSecondCube(DriveTrain dt, Intake intake) {
    this.dt = dt;
    this.intake = intake;

    addCommands(
      // new ToggleIntakeSolenoid(intake),

      // new Delay(0.5),

      new ParallelDeadlineGroup(
        new frc.robot.commands.AutoCommands.DriveDistance(dt, 2050),
        new RunIntake(intake)
      ),

      new ParallelDeadlineGroup(
        new Delay(0.25),
        new RunIntake(intake)
      )

      //new frc.robot.commands.AutoCommands.DriveDistance(dt, -2050)


    );
  }
}
