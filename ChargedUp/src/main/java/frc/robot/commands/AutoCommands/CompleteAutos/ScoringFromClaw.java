// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.CompleteAutos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.HelperCommands.ExtendAndSuck;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ExtensionArms;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoringFromClaw extends SequentialCommandGroup {
  /** Creates a new ScoringFromClaw. */
  public ScoringFromClaw(Claw claw, ExtensionArms extensionArms, Arms arms) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ExtendAndSuck(claw,extensionArms,arms));
  }
}
