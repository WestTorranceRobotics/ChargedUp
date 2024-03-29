// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.HelperCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClawOutward;
import frc.robot.commands.ClawSolenoid;
import frc.robot.commands.ToggleExtensionArmSetpoint;
import frc.robot.commands.ToggleIntakeSolenoid;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ExtensionArms;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CubeScoringAutonomous extends SequentialCommandGroup {
  /** Creates a new CubeScoringAutonomous. */
  public CubeScoringAutonomous(Claw claw, ExtensionArms extensionArms, Arms arms, Intake intake) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( new ToggleIntakeSolenoid(intake),
    new ExtendAndSuck(claw, extensionArms, arms),
    new ParallelDeadlineGroup(new EndWhenRotatedToPoint(arms,-54),new RunArmPositionAuto(arms, -54)),
    new ParallelDeadlineGroup(new EndWhenExtendedToPoint(extensionArms,-100),new ToggleExtensionArmSetpoint(extensionArms, arms, 1)),    
    new ParallelDeadlineGroup(new Delay(0.25), new ClawOutward(claw), new ClawSolenoid(claw, false)),
    new ParallelDeadlineGroup(new EndWhenExtendedToPoint(extensionArms, -1), new ToggleExtensionArmSetpoint(extensionArms, arms, 0))
    );
  }
}
