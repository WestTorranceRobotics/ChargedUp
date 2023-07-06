// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClawInward;
import frc.robot.commands.ClawOutward;
import frc.robot.commands.ClawOutwardSlow;
import frc.robot.commands.ToggleArmSetpoint;
import frc.robot.commands.AutoCommands.HelperCommands.Delay;
import frc.robot.commands.AutoCommands.HelperCommands.EndWhenRotatedToPoint;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ExtensionArms;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceHighCube extends SequentialCommandGroup {
  /** Creates a new PlaceMidCube. */
  public PlaceHighCube(Arms arms, Claw claw, ExtensionArms extArms) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new CustomArmSetPoint(arms, -60),
      // new ToggleArmSetpoint(arms, 0, null)
      new ParallelDeadlineGroup(
        new EndWhenRotatedToPoint(arms, -53.2),
        new ToggleArmSetpoint(arms, 2, extArms)
      ),

      new Delay(0.2),

      new ParallelDeadlineGroup(
        new Delay(0.5),
        new ClawInward(claw)
      ),
      new ParallelDeadlineGroup(
        new EndWhenRotatedToPoint(arms, -1),
        new ToggleArmSetpoint(arms, 0, extArms)
      )
    );
  }
}
