// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.CompleteAutos;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClawInward;
import frc.robot.commands.ClawOutward;
import frc.robot.commands.ClawSolenoid;
import frc.robot.commands.RunArmPosition;
import frc.robot.commands.ToggleArmSetpoint;
import frc.robot.commands.ToggleExtensionArmSetpoint;
import frc.robot.commands.ToggleIntakeSolenoid;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExtensionArms;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AutoCommands.HelperCommands.*;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoCubeAutonomous extends ParallelCommandGroup {
  /** Creates a new TwoCubeAutonomous. */
  public TwoCubeAutonomous(Claw claw, ExtensionArms extensionArms, Arms arms, DriveTrain driveTrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SequentialCommandGroup(
      new SetIsAuto(claw, true),
      new DisplayStartingGyro(driveTrain),
      new InstantCommand(driveTrain::setStartYawOffset),
      //new ToggleIntakeSolenoid(intake),
      new ExtendAndSuckCube(claw, extensionArms, arms),
      new ParallelDeadlineGroup(new EndWhenRotatedToPoint(arms,-53),new ClawPassive(claw),new RunArmPositionAuto(arms, -53), new ClawPassive(claw)),
      new ParallelDeadlineGroup(new EndWhenExtendedToPoint(extensionArms,-95),new ClawPassive(claw),new ToggleExtensionArmSetpoint(extensionArms, arms, 1)),    
      new ParallelDeadlineGroup(new Delay(0.2), new ClawActive(claw, 0.2)),
      new ParallelDeadlineGroup(new EndWhenExtendedToPoint(extensionArms, 0),new ToggleExtensionArmSetpoint(extensionArms, arms,0)),
      //new ScoringSecondCube(claw, extensionArms, arms, intake, driveTrain),
      //new ParallelDeadlineGroup(new EndWhenRotatedToPoint(arms, 0),new RunArmPositionAuto(arms, 0)),
      new frc.robot.commands.DriveTrain.DriveDistance(driveTrain, 2050),


      new SetIsAuto(claw, false)
    //new ParallelDeadlineGroup(new EndWhenRotatedToPoint(arms, -25), new RunArmPositionAuto(arms, -25))
    ));
  }
}

