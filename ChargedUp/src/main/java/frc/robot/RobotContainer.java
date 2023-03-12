// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.ExtensionArms;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;

import frc.robot.commands.TankDrive;
import frc.robot.commands.ControllerTankDrive;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ClawInward;
import frc.robot.commands.ClawOutward;
import frc.robot.commands.PointToLime;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunOuttake;
import frc.robot.commands.SpindexerClockwise;
import frc.robot.commands.SpindexerCounterclockwise;
import frc.robot.commands.ClawSolenoid;
import frc.robot.commands.ClawTurning;
import frc.robot.commands.RunArmPosition;
import frc.robot.commands.RunArmPower;
import frc.robot.commands.RunExtensionArmPosition;
import frc.robot.commands.ToggleArmSetpoint;
import frc.robot.commands.ToggleExtensionArmSetpoint;
import frc.robot.commands.ToggleIntakeSolenoid;
import frc.robot.commands.Auto.GyroBalance;
import frc.robot.commands.Test.FieldOrientedDrive;
import frc.robot.commands.Test.FieldOrientedDrive;
import frc.robot.RobotMap.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
public class RobotContainer {
  
  Arms armSubsystem;
  ExtensionArms extensionArmSubsystem;
  DriveTrain driverBaseSubsystem;
  LimeLight limelightSubsystem;
  Intake intakesubsystem;
  Spindexer spindexerSubsystem;
  Claw clawSubsystem;

  ControllerTankDrive controllerTankDrive;
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
  RunArmPower runArmPower;
  ToggleArmSetpoint startingArmSetpoint;
  ToggleArmSetpoint rightPerpendicularArmSetpoint;
  ToggleArmSetpoint leftPerpendicularArmSetpoint;
  ToggleArmSetpoint leftFourtyFiveArmSetpoint;

  ToggleExtensionArmSetpoint startingExtensionArmSetpoint;
  ToggleExtensionArmSetpoint midExtensionArmSetpoint;
  ToggleExtensionArmSetpoint highExtensionArmSetpoint;


  
  RunExtensionArmPosition runExtensionArmPosition;

  SpindexerClockwise spindexerClockwise;
  SpindexerCounterclockwise spindexerCounterClockwise;


  private static final XboxController xboxController = new XboxController(1);
  private static final Joystick leftJoystick = new Joystick(RobotMap.JoyStickConstants.leftJoystickPort);
  private static final Joystick rightJoystick = new Joystick(RobotMap.JoyStickConstants.rightJoystickPort);
  private  JoystickButton driverLeftTrigger = new JoystickButton(leftJoystick,RobotMap.JoyStickConstants.leftJoystickTrigger );
  private JoystickButton driverRightTrigger = new JoystickButton(rightJoystick, RobotMap.JoyStickConstants.rightJoystickTrigger);
  
  private JoystickButton operatorXbutton = new JoystickButton(xboxController, 1);
  private JoystickButton operatorAbutton = new JoystickButton(xboxController, 2);
  private JoystickButton operatorBbutton = new JoystickButton(xboxController, 3);
  private JoystickButton operatorYbutton = new JoystickButton(xboxController, 4);
  private JoystickButton operatorLeftBack = new JoystickButton(xboxController, 5);
  private JoystickButton operatorRightBack = new JoystickButton(xboxController, 6);
  private JoystickButton operatorLeftTrigger = new JoystickButton(xboxController, 7);
  private JoystickButton operatorRightTrigger = new JoystickButton(xboxController, 8);
  private JoystickButton operatorBack = new JoystickButton(xboxController, 9);
  private JoystickButton operatorStart = new JoystickButton(xboxController, 10);

  private POVButton operatorPOV90 = new POVButton(xboxController, 90);
  private POVButton operatorPOV180 = new POVButton(xboxController, 180);
  private POVButton operatorPOV270 = new POVButton(xboxController, 270);
  private POVButton operatorPOV360 = new POVButton(xboxController, 360);

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

  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController m_driverController =
      //new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
 
 
 
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
      armSubsystem = new frc.robot.subsystems.Arms();
    }

    if(RobotMap.enableExtensionArm){
      extensionArmSubsystem = new frc.robot.subsystems.ExtensionArms();
    }
    
  }

  private void initCommands(){

    if (RobotMap.enableDrivetrain ){
      driveBaseArcadeDriveCommand = new ArcadeDrive(xboxController, driverBaseSubsystem);
      driveBaseTankDriveCommand = new TankDrive(leftJoystick, rightJoystick, driverBaseSubsystem);
      controllerTankDrive = new ControllerTankDrive(xboxController, driverBaseSubsystem);
      // driverBaseSubsystem.setDefaultCommand(driveBaseTankDriveCommand);
      // driverBaseSubsystem.setDefaultCommand(controllerTankDrive);
      // driverBaseSubsystem.setDefaultCommand(new FieldOrientedDrive(xboxController, driverBaseSubsystem));
      driverBaseSubsystem.setDefaultCommand(driveBaseArcadeDriveCommand);

      operatorXbutton.whileTrue(new GyroBalance(driverBaseSubsystem));
    }

    if (RobotMap.enableClaw){
      clawTurningClockwise = new ClawTurning(clawSubsystem, true);
      clawTurningCounterClockwise = new ClawTurning(clawSubsystem, false);
      clawsolenoidExtend = new ClawSolenoid(clawSubsystem, true);
      clawsolenoidRetract = new ClawSolenoid(clawSubsystem, false);
      clawInward = new ClawInward(clawSubsystem);
      clawOutward = new ClawOutward(clawSubsystem);

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

      runArmPower = new RunArmPower(armSubsystem);
      startingArmSetpoint = new ToggleArmSetpoint(armSubsystem, 0);
      rightPerpendicularArmSetpoint  = new ToggleArmSetpoint(armSubsystem, 1);
      leftPerpendicularArmSetpoint = new ToggleArmSetpoint(armSubsystem, 2);
      leftFourtyFiveArmSetpoint = new ToggleArmSetpoint(armSubsystem, 3);
      
    }

    if(RobotMap.enableExtensionArm){
      runExtensionArmPosition = new RunExtensionArmPosition(extensionArmSubsystem, xboxController);
      extensionArmSubsystem.setDefaultCommand(runExtensionArmPosition);
      startingExtensionArmSetpoint = new ToggleExtensionArmSetpoint(extensionArmSubsystem, 0);
      midExtensionArmSetpoint = new ToggleExtensionArmSetpoint(extensionArmSubsystem, 1);
      highExtensionArmSetpoint = new ToggleExtensionArmSetpoint(extensionArmSubsystem, 2);
      
    }
 

  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    //Operator -Ask Ishan
    if (RobotMap.enableArm){
    operatorLeftTrigger.whileTrue(startingArmSetpoint);
    operatorLeftBack.whileTrue(rightPerpendicularArmSetpoint);
    operatorRightTrigger.whileTrue(leftPerpendicularArmSetpoint);
    operatorRightBack.whileTrue(leftFourtyFiveArmSetpoint);
    }
   
    if (RobotMap.enableClaw){
    operatorXbutton.whileTrue(clawInward);
    operatorBbutton.whileTrue(clawOutward);
    operatorYbutton.whileTrue(clawsolenoidExtend);
    operatorAbutton.whileTrue(clawsolenoidRetract);
    //operatorPOV180.whileTrue(clawTurningClockwise);
    //operatorPOV360.whileTrue(clawTurningCounterClockwise);
    }

    if (RobotMap.enableSpindexer){
    operatorStart.whileTrue(spindexerClockwise);
    operatorBack.whileTrue(spindexerCounterClockwise);
    }

    if (RobotMap.enableExtensionArm){
      operatorPOV90.whileTrue(startingArmSetpoint);
      operatorPOV270.whileTrue(highExtensionArmSetpoint);
    }

    if (RobotMap.enableIntake){
      driverLeftTrigger.whileTrue(runOuttakeCommand);
      driverRightTrigger.whileTrue(runIntakeCommand);
      driverRightTopLeft.whileTrue(toggleIntakeSolenoid);
      
    }

    if ((RobotMap.enableDrivetrain) && (RobotMap.enableLimelight)){
      operatorAbutton.whileTrue(pointToLimeCommand);
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
