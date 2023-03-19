// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RunExtensionArmPosition;
import frc.robot.commands.ToggleArmSetpoint;
import frc.robot.commands.ToggleExtensionArmSetpoint;
import frc.robot.commands.AutoCommands.HelperCommands.EndWhenRotatedToPoint;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ExtensionArms;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceHigh extends SequentialCommandGroup {
  /** Creates a new PlaceHigh. */
  public PlaceHigh(Claw claw, ExtensionArms extArm, Arms arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands
    (
      new ToggleArmSetpoint(arm, 2),
      new ParallelCommandGroup
      (
        new ToggleClaw(claw), 
        new SequentialCommandGroup 
        (
          new EndWhenRotatedToPoint(arm, -49),
          new ToggleExtensionArmSetpoint(extArm,arm, 1)
        )
      )
    );
  }
}
