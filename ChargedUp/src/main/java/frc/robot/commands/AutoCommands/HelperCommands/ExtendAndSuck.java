// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.HelperCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClawInward;
import frc.robot.commands.ToggleExtensionArmSetpoint;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ExtensionArms;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExtendAndSuck extends SequentialCommandGroup {
  /** Creates a new ExtendAndSuck. */
  public ExtendAndSuck(Claw claw, ExtensionArms extensionArms, Arms arms) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ToggleExtensionArmSetpoint(extensionArms, arms, 2),

      new ParallelDeadlineGroup(
        new EndWhenExtendedToPoint(extensionArms, -12),
        new ClawInward(claw)
      ),
      
      // new ParallelCommandGroup(
      //   new Delay(0.5),
      //   new ClawInward(claw)
      // ),
      
      new ToggleExtensionArmSetpoint(extensionArms, arms, 0)

    );

  }
}
