//#region Imports
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ExtensionArms;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;

import frc.robot.commands.TankDrive;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ClawInward;
import frc.robot.commands.ClawLSRotate;
import frc.robot.commands.ClawOutward;
import frc.robot.commands.ClawRotation;
import frc.robot.commands.PointToLime;
import frc.robot.commands.RotateArmDefualt;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunOuttake;
import frc.robot.commands.SpindexerClockwise;
import frc.robot.commands.SpindexerCounterclockwise;
import frc.robot.commands.ClawSolenoid;
import frc.robot.commands.ClawSpeed;
import frc.robot.commands.ClawTurning;
import frc.robot.commands.ExtensionArmDefualt;
import frc.robot.commands.RunArmPosition;
import frc.robot.commands.RunArmPower;
import frc.robot.commands.RunExtensionArmPosition;
import frc.robot.commands.RunExtensionArmPower;
import frc.robot.commands.ToggleArmSetpoint;
import frc.robot.commands.ToggleExtensionArmSetpoint;
import frc.robot.commands.ToggleIntakeSolenoid;
import frc.robot.commands.Test.WorkingTurnInPlace;
import frc.robot.RobotMap.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.fasterxml.jackson.databind.util.PrimitiveArrayBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.ADXL345_I2C.Axes;
import edu.wpi.first.wpilibj.XboxController.Axis;


//#endregion

public class RobotContainer {
  //#region Variable/Object Def
  Arm armSubsystem;
  ExtensionArms extensionArmSubsystem;
  DriveTrain driverBaseSubsystem;
  LimeLight limelightSubsystem;
  Intake intakesubsystem;
  Spindexer spindexerSubsystem;
  Claw clawSubsystem;


  TankDrive driveBaseTankDriveCommand;
  ArcadeDrive driveBaseArcadeDriveCommand;
  RunIntake runIntakeCommand;
  RunOuttake runOuttakeCommand;
  ToggleIntakeSolenoid toggleIntakeSolenoid;
  PointToLime pointToLimeCommand;
  ClawSolenoid clawsolenoidExtend;
  ClawSolenoid clawsolenoidRetract;
  ClawTurning clawTurningClockwise;
  ClawTurning clawTurningCounterClockwise; 
  ClawInward clawInward;
  ClawOutward clawOutward; 
  RunArmPosition runArmPosition;
  // RunArmPower runArmPower;
  ToggleArmSetpoint startingArmSetpoint;
  ToggleArmSetpoint rightPerpendicularArmSetpoint;
  ToggleArmSetpoint leftPerpendicularArmSetpoint;
  ToggleArmSetpoint leftFourtyFiveArmSetpoint;

  ToggleExtensionArmSetpoint startingExtensionArmSetpoint;
  ToggleExtensionArmSetpoint midExtensionArmSetpoint;
  ToggleExtensionArmSetpoint highExtensionArmSetpoint;

  RunExtensionArmPower runExtensionArmPower;
  ClawSpeed clawSpeed;
  
  RunExtensionArmPosition runExtensionArmPosition;

  SpindexerClockwise spindexerClockwise;
  SpindexerCounterclockwise spindexerCounterClockwise;

  ClawRotation clawRotation;


  private static final XboxController xboxController = new XboxController(2);
  private static final Joystick leftJoystick = new Joystick(RobotMap.JoyStickConstants.leftJoystickPort);
  private static final Joystick rightJoystick = new Joystick(RobotMap.JoyStickConstants.rightJoystickPort);
  private  JoystickButton driverLeftTrigger = new JoystickButton(leftJoystick,RobotMap.JoyStickConstants.leftJoystickTrigger );
  private JoystickButton driverRightTrigger = new JoystickButton(rightJoystick, RobotMap.JoyStickConstants.rightJoystickTrigger);
  
  private JoystickButton operatorXbutton = new JoystickButton(xboxController, 1);
  private JoystickButton operatorAbutton = new JoystickButton(xboxController, 2);
  private JoystickButton operatorBbutton = new JoystickButton(xboxController, 3);
  private JoystickButton operatorYbutton = new JoystickButton(xboxController, 4);
  private JoystickButton operatorLeftBumper = new JoystickButton(xboxController, 5);
  private JoystickButton operatorRightBumper = new JoystickButton(xboxController, 6);
  //private JoystickButton operatorLeftTrigger = new JoystickButton(xboxController, 7);
  //private JoystickButton operatorRightTrigger = new JoystickButton(xboxController, 8);
  private JoystickButton operatorBack = new JoystickButton(xboxController, 7);
  private JoystickButton operatorStart = new JoystickButton(xboxController, 8);

  

  private POVButton operatorPOVRight = new POVButton(xboxController, 90);
  private POVButton operatorPOVDown = new POVButton(xboxController, 180);
  private POVButton operatorPOVLeft = new POVButton(xboxController, 270);
  private POVButton operatorPOVUp = new POVButton(xboxController, 0);

  private POVButton operatorPOVUpRight = new POVButton(xboxController, 45);
  private POVButton operatorPOVUpLeft = new POVButton(xboxController, 315);
  private POVButton operatorPOVDownRight = new POVButton(xboxController, 135);
  private POVButton operatorPOVDownLeft = new POVButton(xboxController, 225);

  private JoystickButton driverLeftTopLeft = new JoystickButton(leftJoystick, 5);
  private JoystickButton driverLeftTopRight = new JoystickButton(leftJoystick, 6);
  private JoystickButton driverLeftBottomLeft = new JoystickButton(leftJoystick, 3);
  private JoystickButton driverLeftBottomRight = new JoystickButton(leftJoystick, 4);
  private JoystickButton driverLeftSide = new JoystickButton(leftJoystick, 2);

  private JoystickButton driverRightTopLeft = new JoystickButton(rightJoystick, 5);
  private JoystickButton driverRightTopRight = new JoystickButton(rightJoystick, 6);
  private JoystickButton driverRightBottomLeft = new JoystickButton(rightJoystick, 3);
  private JoystickButton driverRightBottomRight = new JoystickButton(rightJoystick, 4);
  private JoystickButton driverRightSide = new JoystickButton(rightJoystick, 2);

  
  // The robot's subsystems and commands are defined here...


 // XboxController controller = new XboxController(1);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController m_driverController =
      //new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
 
  //#endregion
 
  public RobotContainer() {
    
    initSubsytems();
    initCommands();
    // Configure the trigger bindings
    configureBindings();
  }

  private void initSubsytems() {
    if (RobotMap.enableDrivetrain ){
      driverBaseSubsystem = new frc.robot.subsystems.DriveTrain();
    }

    if (RobotMap.enableClaw){
      clawSubsystem = new frc.robot.subsystems.Claw();
    }

    if (RobotMap.enableIntake){
      intakesubsystem = new frc.robot.subsystems.Intake();
    }

    if(RobotMap.enableLimelight){
      limelightSubsystem = new frc.robot.subsystems.LimeLight();
    }

    if(RobotMap.enableSpindexer){
      spindexerSubsystem = new frc.robot.subsystems.Spindexer();
    }

    if (RobotMap.enableArm){
      armSubsystem = new frc.robot.subsystems.Arm();
    }

    if(RobotMap.enableExtensionArm){
      extensionArmSubsystem = new frc.robot.subsystems.ExtensionArms();
    }
    
  }

  private void initCommands(){

    if (RobotMap.enableDrivetrain ){
      driveBaseArcadeDriveCommand = new ArcadeDrive(xboxController, driverBaseSubsystem);
      driveBaseTankDriveCommand = new TankDrive(leftJoystick, rightJoystick, driverBaseSubsystem);
      // driverBaseSubsystem.setDefaultCommand(driveBaseArcadeDriveCommand);
      driverBaseSubsystem.setDefaultCommand(new WorkingTurnInPlace(xboxController, driverBaseSubsystem));
    }

    if (RobotMap.enableClaw){
      clawSpeed = new ClawSpeed(clawSubsystem);
      clawTurningClockwise = new ClawTurning(clawSubsystem, true);
      clawTurningCounterClockwise = new ClawTurning(clawSubsystem, false);
      clawsolenoidExtend = new ClawSolenoid(clawSubsystem, true);
      clawsolenoidRetract = new ClawSolenoid(clawSubsystem, false);
      clawInward = new ClawInward(clawSubsystem);
      clawOutward = new ClawOutward(clawSubsystem);
      //clawRotation = new ClawRotation(clawSubsystem, xboxController);
      //clawSubsystem.setDefaultCommand(clawRotation);

    }

    if (RobotMap.enableIntake){
      toggleIntakeSolenoid = new ToggleIntakeSolenoid(intakesubsystem);
      runIntakeCommand = new RunIntake(intakesubsystem);
      runOuttakeCommand = new RunOuttake(intakesubsystem);
      

    }

    if(RobotMap.enableLimelight && RobotMap.enableDrivetrain){
      pointToLimeCommand = new PointToLime(driverBaseSubsystem, limelightSubsystem);
    }

    if(RobotMap.enableSpindexer){
      spindexerClockwise = new SpindexerClockwise(spindexerSubsystem);
      spindexerCounterClockwise = new SpindexerCounterclockwise(spindexerSubsystem);
      
    }

    if (RobotMap.enableArm){
      
      runArmPosition = new RunArmPosition(armSubsystem,xboxController);
      armSubsystem.setDefaultCommand(runArmPosition);
      // runArmPower = new RunArmPower(armSubsystem);
      startingArmSetpoint = new ToggleArmSetpoint(armSubsystem, 0);
      rightPerpendicularArmSetpoint  = new ToggleArmSetpoint(armSubsystem, 1);
      leftPerpendicularArmSetpoint = new ToggleArmSetpoint(armSubsystem, 2);
      leftFourtyFiveArmSetpoint = new ToggleArmSetpoint(armSubsystem, 3);
      
    }

    if(RobotMap.enableExtensionArm){
      runExtensionArmPower = new RunExtensionArmPower(extensionArmSubsystem);
      runExtensionArmPosition = new RunExtensionArmPosition(extensionArmSubsystem, xboxController);
      // extensionArmSubsystem.setDefaultCommand(runExtensionArmPosition);
      startingExtensionArmSetpoint = new ToggleExtensionArmSetpoint(extensionArmSubsystem, 0);
      highExtensionArmSetpoint = new ToggleExtensionArmSetpoint(extensionArmSubsystem, 1);
            
    }
 

  }

  private void configureBindings() {

    if (RobotMap.enableArm){
      armSubsystem.setDefaultCommand(new RotateArmDefualt(armSubsystem, xboxController));
    }
   
    if (RobotMap.enableClaw){
      operatorPOVLeft.whileTrue(clawInward);
      operatorPOVUpLeft.whileTrue(clawInward);
      operatorPOVDownLeft.whileTrue(clawInward);

      operatorPOVRight.whileTrue(clawOutward);
      operatorPOVUpRight.whileTrue(clawOutward);
      operatorPOVDownRight.whileTrue(clawOutward);

      operatorAbutton.onTrue(new ClawLSRotate(clawSubsystem));
      operatorBbutton.toggleOnTrue(clawsolenoidExtend);
    
    }

    if (RobotMap.enableSpindexer){
      operatorXbutton.whileTrue(spindexerClockwise);
    }

    if (RobotMap.enableExtensionArm){
      extensionArmSubsystem.setDefaultCommand(new ExtensionArmDefualt(extensionArmSubsystem, xboxController));
    }

    if (RobotMap.enableIntake){
      operatorYbutton.whileTrue(runOuttakeCommand);
      operatorRightBumper.whileTrue(runIntakeCommand);
      operatorLeftBumper.onTrue(toggleIntakeSolenoid);
      
    }

    if ((RobotMap.enableDrivetrain) && (RobotMap.enableLimelight)){
      operatorStart.whileTrue(pointToLimeCommand);
    }

    


    //operatorPOV90.whileTrue(increaseExtensionArmSetpoint);
    //operatorPOV270.whileTrue(decreaseExtensionArmSetpoint);
    //operatorPOV180.whileTrue(clawTurningClockwise);
    //operatorPOV360.whileTrue(clawTurningCounterClockwise);
    //operatorStart.whileTrue(spindexerClockwise);
    //operatorBack.whileTrue(spindexerCounterClockwise);
    




   

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   */
  //public Command getAutonomousCommand() {
    // An example command will be run in autonomous

  }
//}
