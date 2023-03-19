// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.HelperCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClawSolenoid;
import frc.robot.commands.ToggleArmSetpoint;
import frc.robot.commands.ToggleExtensionArmSetpoint;
import frc.robot.commands.Limelight.PointToLime;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExtensionArms;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceConeTop extends SequentialCommandGroup {
  /** Creates a new PlaceConeTop. */
  public PlaceConeTop(DriveTrain dt, LimeLight limeLight, Arms arms, ExtensionArms extensionArms, Claw claw, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PointToLime(dt, limeLight),
      new ParallelDeadlineGroup(new EndWhenRotatedToPoint(arms,-50),new ClawPassive(claw),new RunArmPositionAuto(arms, -50)),
      new ParallelDeadlineGroup(new EndWhenExtendedToPoint(extensionArms,-95),new ClawPassive(claw),new ToggleExtensionArmSetpoint(extensionArms, arms, 1)),    
      new ParallelDeadlineGroup(new Delay(0.25),new ClawSolenoid(claw, false)),
      new ParallelDeadlineGroup(new EndWhenExtendedToPoint(extensionArms, 0)  ,new ToggleExtensionArmSetpoint(extensionArms, arms, 0)),
      new ParallelCommandGroup(new EndWhenRotatedToPoint(arms, 0), new ToggleArmSetpoint(arms, 0))
    );
  }
}
