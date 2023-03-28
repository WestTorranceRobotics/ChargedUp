// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClawInward;
import frc.robot.commands.ClawOutward;
import frc.robot.commands.ClawSolenoid;
import frc.robot.commands.RunExtensionArmPosition;
import frc.robot.commands.ToggleArmSetpoint;
import frc.robot.commands.ToggleExtensionArmSetpoint;
import frc.robot.commands.ToggleIntakeSolenoid;
import frc.robot.commands.AutoCommands.HelperCommands.ClawPassive;
import frc.robot.commands.AutoCommands.HelperCommands.Delay;
import frc.robot.commands.AutoCommands.HelperCommands.EndWhenExtendedToPoint;
import frc.robot.commands.AutoCommands.HelperCommands.EndWhenRotatedToPoint;
import frc.robot.commands.AutoCommands.HelperCommands.RunArmPositionAuto;
import frc.robot.commands.AutoCommands.HelperCommands.RunExtensionArmPositionAuto;
import frc.robot.commands.Claw.ToggleClaw;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ExtensionArms;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClawScoreHigh extends SequentialCommandGroup {
  /** Creates a new ClawScoreHigh. */
  public ClawScoreHigh(Claw claw, ExtensionArms extensionArms, Arms arms) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(new EndWhenRotatedToPoint(arms,-25),new ClawPassive(claw),new RunArmPositionAuto(arms, -50)),
      new EndWhenRotatedToPoint(arms, -45),
      new ParallelDeadlineGroup(new EndWhenExtendedToPoint(extensionArms,-90),new ClawPassive(claw),new RunExtensionArmPositionAuto(extensionArms, -95))
     );
  }
}
