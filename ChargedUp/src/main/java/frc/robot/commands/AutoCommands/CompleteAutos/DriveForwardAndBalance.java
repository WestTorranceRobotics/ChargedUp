// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.CompleteAutos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.HelperCommands.HoldInPlacePID;
import frc.robot.commands.DriveTrain.DriveDistance;
//import frc.robot.commands.AutoCommands.DriveTrain.DriveDistance;
import frc.robot.commands.DriveTrain.DriveDistancePID;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveForwardAndBalance extends SequentialCommandGroup {
  /** Creates a new DriveForwardAndBalance. */
  public DriveForwardAndBalance(DriveTrain driveTrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new DriveDistancePID(driveTrain,1150)
    new DriveDistance(driveTrain, 2070),
    new DriveDistance(driveTrain, -1185),
    new HoldInPlacePID(driveTrain)
    );
  }
}
