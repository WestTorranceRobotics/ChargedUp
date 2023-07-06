// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ExtArm;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ToggleArmSetpoint;
import frc.robot.commands.ToggleExtensionArmSetpoint;
import frc.robot.commands.AutoCommands.HelperCommands.Delay;
import frc.robot.commands.AutoCommands.HelperCommands.EndWhenExtendedToPoint;
import frc.robot.commands.AutoCommands.HelperCommands.EndWhenRotatedToPoint;
import frc.robot.commands.AutoCommands.HelperCommands.RunExtensionArmPositionAuto;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.ExtensionArms;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StartingPositionRetract extends SequentialCommandGroup {
  /** Creates a new HumanPlayer. */
  public StartingPositionRetract(ExtensionArms extArms, Arms arms) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(      
      new ToggleExtensionArmSetpoint(extArms, arms, 4),
      // new EndWhenExtendedToPoint(extArms, -5),
      // new Delay(0.5),
      new ToggleArmSetpoint(arms, 0, extArms),
      new EndWhenRotatedToPoint(arms, -1)
      
    );
  }
}
