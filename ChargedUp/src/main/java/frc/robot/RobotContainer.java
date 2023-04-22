// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.ExtensionArms;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
import frc.robot.commands.ClawInward;
import frc.robot.commands.ClawOutward;
import frc.robot.commands.ClawRotation;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunOuttake;
import frc.robot.commands.SpindexerClockwise;
import frc.robot.commands.SpindexerCounterclockwise;
import frc.robot.commands.ClawSolenoid;
import frc.robot.commands.ClawSpeed;
import frc.robot.commands.BackwardsInvertedArcade;
import frc.robot.commands.RotateAndTwist;
//import frc.robot.commands.ClawTurning;
import frc.robot.commands.RunArmPosition;
import frc.robot.commands.RunArmPower;
import frc.robot.commands.RunExtensionArmPosition;
import frc.robot.commands.RunExtensionArmPower;
import frc.robot.commands.ToggleArmSetpoint;
import frc.robot.commands.ToggleExtensionArmSetpoint;
import frc.robot.commands.ToggleIntakeSolenoid;
import frc.robot.commands.DriveTrain.ArcadeDrive;
import frc.robot.commands.DriveTrain.DriveDistance;
import frc.robot.commands.DriveTrain.DriveStraight;
import frc.robot.commands.DriveTrain.DriveTrainDefualt;
import frc.robot.commands.DriveTrain.NonPIDChargeStationBalance;
import frc.robot.commands.DriveTrain.TankDrive;
import frc.robot.commands.DriveTrain.TankDriveController;
import frc.robot.commands.AutoCommands.ClawScoreHigh;
// import frc.robot.commands.DriveTrain.NonPIDChargeStationBalance;
// import frc.robot.commands.DriveTrain.PIDChargeStationBalance;
import frc.robot.commands.AutoCommands.CompleteAutos.BasicGrabSecondCube;
import frc.robot.commands.AutoCommands.CompleteAutos.DriveForwardAndBalance;
import frc.robot.commands.AutoCommands.CompleteAutos.ScoringSecondCube;
import frc.robot.commands.AutoCommands.CompleteAutos.TwoCubeAutonomous;
import frc.robot.commands.AutoCommands.HelperCommands.ConeScoringAutonomous;
import frc.robot.commands.AutoCommands.HelperCommands.CubeScoringAutonomous;
import frc.robot.commands.AutoCommands.HelperCommands.ExtendAndSuck;
import frc.robot.commands.AutoCommands.HelperCommands.ExtendAndSuckCube;
import frc.robot.commands.AutoCommands.HelperCommands.PlaceConeTop;
import frc.robot.commands.Claw.CloseIntakeClaw;
import frc.robot.commands.Limelight.LimelightAlignWithGyro;
import frc.robot.commands.Limelight.PointToLime;
import frc.robot.commands.Test.TurnToDirection;
import frc.robot.RobotMap.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.fasterxml.jackson.databind.util.PrimitiveArrayBuilder;

import edu.wpi.first.networktables.EntryBase;
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
public class RobotContainer {
  
  Arms armSubsystem;
  ExtensionArms extensionArmSubsystem;
  DriveTrain driverBaseSubsystem;
  LimeLight limelightSubsystem;
  Intake intakesubsystem;
  Spindexer spindexerSubsystem;
  Claw clawSubsystem;
  LEDs leds;


  TankDrive driveBaseTankDriveCommand;
  ArcadeDrive driveBaseArcadeDriveCommand;
  RunIntake intakeCommand;
  RunOuttake outtakeCommand;
  ToggleIntakeSolenoid toggleIntakeSolenoid;
  PointToLime pointToLimeCommand;
  ClawSolenoid clawsolenoidExtend;
  ClawSolenoid clawsolenoidRetract;
 // ClawTurning clawTurningClockwise;
  //ClawTurning clawTurfningCounterClockwise; 
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

  RunExtensionArmPower runExtensionArmPower;
  ClawSpeed clawSpeed;
  
  RunExtensionArmPosition runExtensionArmPosition;

  SpindexerClockwise spindexerClockwise;
  SpindexerCounterclockwise spindexerCounterClockwise;

  ClawRotation clawRotation;

  ExtendAndSuck extendAndSuck;
  ExtendAndSuckCube extendAndSuckCube;
  CubeScoringAutonomous cubeScoringAutonomous;
  ConeScoringAutonomous coneScoringAutonomous;
  BasicGrabSecondCube basicGrabSecondCube;
  RotateAndTwist rotateBackandTwist;
  RotateAndTwist rotateFrontRotateAndTwist;
  ClawScoreHigh clawScoreHigh;

  private static final XboxController operatorController = new XboxController(3);
  private static final XboxController driverxboxController = new XboxController(4);

  //private static final Joystick leftJoystick = new Joystick(RobotMap.JoyStickConstants.leftJoystickPort);
  //private static final Joystick rightJoystick = new Joystick(RobotMap.JoyStickConstants.rightJoystickPort);
  //private  JoystickButton driverLeftTrigger = new JoystickButton(leftJoystick,RobotMap.JoyStickConstants.leftJoystickTrigger );
  //private JoystickButton driverRightTrigger = new JoystickButton(rightJoystick, RobotMap.JoyStickConstants.rightJoystickTrigger);
  
  private static final Joystick leftJoystick = new Joystick(0);
private static final Joystick rightJoystick = new Joystick(1);

private JoystickButton driverLeftTopLeft = new JoystickButton(leftJoystick, 5);
private JoystickButton driverLeftTopRight = new JoystickButton(leftJoystick, 6);
private JoystickButton driverLeftBottomLeft = new JoystickButton(leftJoystick, 3);
private JoystickButton driverLeftBottomRight = new JoystickButton(leftJoystick, 4);
private JoystickButton driverLeftSide = new JoystickButton(leftJoystick, 2);
private JoystickButton driverLeftTrigger = new JoystickButton(leftJoystick, 1);

private JoystickButton driverRightTopLeft = new JoystickButton(rightJoystick, 5);
private JoystickButton driverRightTopRight = new JoystickButton(rightJoystick, 6);
private JoystickButton driverRightBottomLeft = new JoystickButton(rightJoystick, 3);
private JoystickButton driverRightBottomRight = new JoystickButton(rightJoystick, 4);
private JoystickButton driverRightSide = new JoystickButton(rightJoystick, 2);
private JoystickButton driverRightTrigger = new JoystickButton(rightJoystick, 1);

  private JoystickButton operatorXbutton = new JoystickButton(operatorController, 1);
  private JoystickButton operatorAbutton = new JoystickButton(operatorController, 2);
  private JoystickButton operatorBbutton = new JoystickButton(operatorController, 3);
  private JoystickButton operatorYbutton = new JoystickButton(operatorController, 4);
  private JoystickButton operatorLeftBack = new JoystickButton(operatorController, 5);
  private JoystickButton operatorRightBack = new JoystickButton(operatorController, 6);
  private JoystickButton operatorLeftTrigger = new JoystickButton(operatorController, 7);
  private JoystickButton operatorRightTrigger = new JoystickButton(operatorController, 8);
  //private JoystickButton operatorBack = new JoystickButton(xboxController, 7);
  private JoystickButton operatorStart = new JoystickButton(operatorController, 10);

  private JoystickButton driverXbutton = new JoystickButton(driverxboxController, 3);
  private JoystickButton driverAbutton = new JoystickButton(driverxboxController, 1);
  private JoystickButton driverBbutton = new JoystickButton(driverxboxController, 2);
  private JoystickButton driverYbutton = new JoystickButton(driverxboxController, 4);
  private JoystickButton driverLeftBumper = new JoystickButton(driverxboxController, 5);
  private JoystickButton driverRightBumper = new JoystickButton(driverxboxController, 6);
  //private JoystickButton operatorLeftTrigger = new JoystickButton(xboxController, 7);
  //private JoystickButton operatorRightTrigger = new JoystickButton(xboxController, 8);
  private JoystickButton driverBack = new JoystickButton(driverxboxController, 7);
  private JoystickButton driverStart = new JoystickButton(driverxboxController, 10);

  private POVButton operatorPOVRight = new POVButton(operatorController, 90);
  private POVButton operatorPOVDown = new POVButton(operatorController, 180);
  private POVButton operatorPOVLeft = new POVButton(operatorController, 270);
  private POVButton operatorPOVUp = new POVButton(operatorController, 0);

  //private JoystickButton driverLeftTopLeft = new JoystickButton(leftJoystick, 5);
  //private JoystickButton driverLeftTopRight = new JoystickButton(leftJoystick, 6);
  //private JoystickButton driverLeftBottomLeft = new JoystickButton(leftJoystick, 3);
  //private JoystickButton driverLeftBottomRight = new JoystickButton(leftJoystick, 4);
  //private JoystickButton driverLeftSide = new JoystickButton(leftJoystick, 2);

  //private JoystickButton driverRightTopLeft = new JoystickButton(rightJoystick, 5);
  //private JoystickButton driverRightTopRight = new JoystickButton(rightJoystick, 6);
  //private JoystickButton driverRightBottomLeft = new JoystickButton(rightJoystick, 3);
  //private JoystickButton driverRightBottomRight = new JoystickButton(rightJoystick, 4);
  //private JoystickButton driverRightSide = new JoystickButton(rightJoystick, 2);






  

  



  // The robot's subsystems and commands are defined here...


 // XboxController controller = new XboxController(1);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController m_driverController =
      //new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
 
 
 
  public RobotContainer() {
    
    initSubsytems();
    initCommands();
    // Configure the trigger bindings
    configure2PersonControl();
  }

  private void initSubsytems() {
    leds = new LEDs();

    if (RobotMap.enableDrivetrain ){
      driverBaseSubsystem = new frc.robot.subsystems.DriveTrain();
    }

    if (RobotMap.enableClaw){
      clawSubsystem = new frc.robot.subsystems.Claw();
    }

    if (RobotMap.enableIntake){
      //intakesubsystem = new frc.robot.subsystems.Intake();
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
    
    if (RobotMap.enableArm && RobotMap.enableClaw){
     // rotateFrontRotateAndTwist = new RotateAndTwist(clawSubsystem, armSubsystem,1);
      //rotateBackandTwist = new RotateAndTwist(clawSubsystem, armSubsystem, 2);
    }
  }

  private void initCommands(){

    if (RobotMap.enableDrivetrain ){
      driveBaseArcadeDriveCommand = new ArcadeDrive(driverxboxController, driverBaseSubsystem);
      driveBaseTankDriveCommand = new TankDrive(leftJoystick,rightJoystick,driverBaseSubsystem);
      // driverBaseSubsystem.setDefaultCommand(new DriveTrainDefualt(driverxboxController, driverBaseSubsystem, limelightSubsystem));
      driverBaseSubsystem.setDefaultCommand(driveBaseTankDriveCommand);  
      // driverBaseSubsystem.setDefaultCommand(driveBaseTankDriveCommand);      
      // driverBaseSubsystem.setDefaultCommand(new BackwardsInvertedArcade(driverxboxController, driverBaseSubsystem));
    }

    if (RobotMap.enableClaw){
      clawSpeed = new ClawSpeed(clawSubsystem);
      //clawTurningClockwise = new ClawTurning(clawSubsystem, true);
      //clawTurningCounterClockwise = new ClawTurning(clawSubsystem, false);
      clawsolenoidExtend = new ClawSolenoid(clawSubsystem, true);
      clawsolenoidRetract = new ClawSolenoid(clawSubsystem, false);
      clawInward = new ClawInward(clawSubsystem);
      clawOutward = new ClawOutward(clawSubsystem);
      clawRotation = new ClawRotation(clawSubsystem, operatorController);
      clawSubsystem.setDefaultCommand(clawRotation);

    }

    if (RobotMap.enableIntake){
      //toggleIntakeSolenoid = new ToggleIntakeSolenoid(intakesubsystem);
      //outtakeCommand = new RunOuttake(intakesubsystem);
      //intakeCommand = new RunIntake(intakesubsystem);
      

    }


    if(RobotMap.enableLimelight && RobotMap.enableDrivetrain){
      pointToLimeCommand = new PointToLime(driverBaseSubsystem, limelightSubsystem);
    }

    if(RobotMap.enableSpindexer){
      spindexerClockwise = new SpindexerClockwise(spindexerSubsystem);
      spindexerCounterClockwise = new SpindexerCounterclockwise(spindexerSubsystem);
      
    }
// && RobotMap.enableIntake
    if (RobotMap.enableArm){
      
      runArmPosition = new RunArmPosition(armSubsystem,operatorController);
      armSubsystem.setDefaultCommand(runArmPosition);
      runArmPower = new RunArmPower(armSubsystem);
      startingArmSetpoint = new ToggleArmSetpoint(armSubsystem, 0,extensionArmSubsystem);
      rightPerpendicularArmSetpoint  = new ToggleArmSetpoint(armSubsystem, 1,extensionArmSubsystem);
      leftPerpendicularArmSetpoint = new ToggleArmSetpoint(armSubsystem, 2,extensionArmSubsystem);
      leftFourtyFiveArmSetpoint = new ToggleArmSetpoint(armSubsystem, 3,extensionArmSubsystem);
      
    }

    if((RobotMap.enableExtensionArm) && (RobotMap.enableArm)){
      runExtensionArmPower = new RunExtensionArmPower(extensionArmSubsystem);
      runExtensionArmPosition = new RunExtensionArmPosition(extensionArmSubsystem, operatorController);
      extensionArmSubsystem.setDefaultCommand(runExtensionArmPosition);
      startingExtensionArmSetpoint = new ToggleExtensionArmSetpoint(extensionArmSubsystem, armSubsystem, 4);
      highExtensionArmSetpoint = new ToggleExtensionArmSetpoint(extensionArmSubsystem,armSubsystem, 1);
            
    }

    if (RobotMap.enableAutonomous){
      extendAndSuck = new ExtendAndSuck(clawSubsystem, extensionArmSubsystem, armSubsystem);
     // cubeScoringAutonomous = new CubeScoringAutonomous(clawSubsystem, extensionArmSubsystem, armSubsystem, intakesubsystem);
      //coneScoringAutonomous = new ConeScoringAutonomous(clawSubsystem, extensionArmSubsystem, armSubsystem, intakesubsystem);
      //basicGrabSecondCube = new BasicGrabSecondCube(driverBaseSubsystem, intakesubsystem);
    }

    if (RobotMap.enableSetpoint){
      extendAndSuckCube = new ExtendAndSuckCube(clawSubsystem, extensionArmSubsystem, armSubsystem);
      clawScoreHigh = new ClawScoreHigh(clawSubsystem, extensionArmSubsystem, armSubsystem);
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

// commented for tank drive

  // private void configure1PersonControl(){
  //   if (RobotMap.enableArm && RobotMap.enableClaw){

  //     operatorPOVDown.whileTrue(startingArmSetpoint);
  //     operatorPOVUp.whileTrue(rightPerpendicularArmSetpoint);
  //     operatorPOVRight.whileTrue(leftPerpendicularArmSetpoint);
  //     operatorPOVLeft.whileTrue(leftFourtyFiveArmSetpoint);
  //   }
   
  //   if (RobotMap.enableClaw){

  //   //Main Command
  //     operatorXbutton.whileTrue(clawOutward);
  //     operatorBbutton.whileTrue(clawInward);
  //     operatorYbutton.onTrue(clawsolenoidExtend);
  //     operatorAbutton.onTrue(clawsolenoidRetract);

  //     driverAbutton.whileTrue(new DriveStraight(driverBaseSubsystem, 0.6));
  //     driverBbutton.whileTrue(new CloseIntakeClaw(clawSubsystem));
  //   }

  //   if (RobotMap.enableExtensionArm && RobotMap.enableArm){
  //     //Main c0mmands
  //     operatorRightBack.whileTrue(highExtensionArmSetpoint);
  //     operatorLeftBack.whileTrue(startingExtensionArmSetpoint);


  //     //Test c0mmands
  //    // operatorPOV90.whileTrue(startingArmSetpoint);
  //     //operatorPOV270.whileTrue(highExtensionArmSetpoint);
  //     //operatorRightBack.whileTrue(runExtensionArmPower);
  //   }

  //   if (RobotMap.enableIntake){
  //     //driverLeftTrigger.whileTrue(outtakeCommand);
  //     //driverRightTrigger.whileTrue(intakeCommand);
  //     //driverRightTopLeft.whileTrue(toggleIntakeSolenoid);
      
  //   }

  //   if ((RobotMap.enableDrivetrain) && (RobotMap.enableLimelight)){
  //     // driverLeftTopRight.toggleOnTrue(pointToLimeCommand);
  //     // driverLeftTopRight.toggleOnTrue(new ClawInward(clawSubsystem));
  //     driverXbutton.toggleOnTrue(new LimelightAlignWithGyro(driverBaseSubsystem, limelightSubsystem));
  //     // driverLeftTopLeft.onTrue(new PlaceConeTop(driverBaseSubsystem, limelightSubsystem, armSubsystem, extensionArmSubsystem, clawSubsystem, intakesubsystem));
  //   }

  //   if (RobotMap.enableAutonomous){
  //     // driverLeftTopLeft.toggleOnTrue(new NonPIDChargeStationBalance(driverBaseSubsystem));
  //     // driverLeftTopLeft.toggleOnTrue(new DriveForwardAndBalance(driverBaseSubsystem));
  //     // driverLeftTopLeft.onTrue(new InstantCommand(driverBaseSubsystem::resetGyro));
  //     // driverLeftBottomRight.onTrue(coneScoringAutonomous);
  //     // driverLeftBottomLeft.onTrue(new TwoCubeAutonomous(clawSubsystem, extensionArmSubsystem, armSubsystem, intakesubsystem, driverBaseSubsystem));
  //     //driverLeftBottomLeft.onTrue(new ScoringSecondCube(clawSubsystem, extensionArmSubsystem, armSubsystem, intakesubsystem, driverBaseSubsystem));
  //     //driverLeftBottomLeft.onTrue(new DriveDistance(driverBaseSubsystem, 2050));
  //   }
  //   if (RobotMap.enableSetpoint){
  //     driverYbutton.onTrue(extendAndSuckCube);
  //     // driverLeftBottomRight.onTrue(new TurnToDirection(driverBaseSubsystem, 0,3));
  //   }

  // }
  private void configure2PersonControl() {

    //Operator -Ask Ishan
    if (RobotMap.enableArm && RobotMap.enableClaw){

      operatorPOVDown.whileTrue(startingArmSetpoint);
      operatorPOVUp.whileTrue(rightPerpendicularArmSetpoint);
      operatorPOVRight.whileTrue(leftPerpendicularArmSetpoint);
      operatorPOVLeft.whileTrue(leftFourtyFiveArmSetpoint);
    }
   
    if (RobotMap.enableClaw){
      driverRightBumper.whileTrue(new CloseIntakeClaw(clawSubsystem));
    //Main Command
      
      operatorXbutton.whileTrue(clawInward);
      operatorBbutton.onTrue(clawsolenoidRetract);
      operatorYbutton.onTrue(clawsolenoidExtend);
      operatorAbutton.whileTrue(clawOutward);
    }


    if (RobotMap.enableSpindexer){
    //operatorStart.whileTrue(spindexerClockwise);
    //operatorBack.whileTrue(spindexerCounterClockwise);
    }

    if (RobotMap.enableExtensionArm && RobotMap.enableArm){
      //Main c0mmands
      operatorRightBack.whileTrue(highExtensionArmSetpoint);
      operatorLeftBack.whileTrue(startingExtensionArmSetpoint);


      //Test c0mmands
     // operatorPOV90.whileTrue(startingArmSetpoint);
      //operatorPOV270.whileTrue(highExtensionArmSetpoint);
      //operatorRightBack.whileTrue(runExtensionArmPower);
    }

    if (RobotMap.enableIntake){
      //driverLeftTrigger.whileTrue(outtakeCommand);
      //driverRightTrigger.whileTrue(intakeCommand);
      //driverRightTopLeft.whileTrue(toggleIntakeSolenoid);
      
    }

    if ((RobotMap.enableDrivetrain) && (RobotMap.enableLimelight)){
      // driverXbutton.toggleOnTrue(pointToLimeCommand);
      // driverXbutton.onFalse(new CloseIntakeClaw(clawSubsystem));
         driverXbutton.onTrue(new LimelightAlignWithGyro(driverBaseSubsystem, limelightSubsystem));
       // driverXbutton.toggleOnTrue(new TurnToDirection(driverBaseSubsystem, 0, 3));
      // driverLeftTopLeft.onTrue(new PlaceConeTop(driverBaseSubsystem, limelightSubsystem, armSubsystem, extensionArmSubsystem, clawSubsystem, intakesubsystem));
    }

    if (RobotMap.enableAutonomous){
      // driverLeftTopLeft.toggleOnTrue(new NonPIDChargeStationBalance(driverBaseSubsystem));
      // driverLeftTopLeft.toggleOnTrue(new DriveForwardAndBalance(driverBaseSubsystem));
      // driverLeftTopLeft.onTrue(new InstantCommand(driverBaseSubsystem::resetGyro));
      // driverLeftBottomRight.onTrue(coneScoringAutonomous);
      // driverLeftBottomLeft.onTrue(new TwoCubeAutonomous(clawSubsystem, extensionArmSubsystem, armSubsystem, intakesubsystem, driverBaseSubsystem));
      //driverLeftBottomLeft.onTrue(new ScoringSecondCube(clawSubsystem, extensionArmSubsystem, armSubsystem, intakesubsystem, driverBaseSubsystem));
      //driverLeftBottomLeft.onTrue(new DriveDistance(driverBaseSubsystem, 2050));
    }
    if (RobotMap.enableSetpoint){
      driverYbutton.onTrue(extendAndSuckCube);
      operatorStart.onTrue(clawScoreHigh);
      // driverLeftBottomRight.onTrue(new TurnToDirection(driverBaseSubsystem, 0,3));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   */
  
 public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return new TwoCubeAutonomous(clawSubsystem, extensionArmSubsystem, armSubsystem, driverBaseSubsystem);
    // return new NonPIDChargeStationBalance(driverBaseSubsystem);
    return new TwoCubeAutonomous(clawSubsystem, extensionArmSubsystem, armSubsystem, driverBaseSubsystem);
 }
 
 public DriveTrain getDriveTrain(){
  return driverBaseSubsystem;
 }
}
