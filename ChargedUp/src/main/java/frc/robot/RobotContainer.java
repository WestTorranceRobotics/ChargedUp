// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.DecreaseArmSetPoint;
import frc.robot.commands.IncreaseArmSetPoint;
import frc.robot.commands.RunArmPosition;
import frc.robot.commands.RunArmPower;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunOuttake;
import frc.robot.commands.ShooterPIDController;
import frc.robot.commands.ShooterSetPower;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.TankDrive;
import frc.robot.RobotMap.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
public class RobotContainer {
  DriveTrain driverBaseSubsystem;
  Intake intakesubsystem;
  Shooter shootersubsystem;
  Arms armsubsystem;
  TankDrive driveBaseTankDriveCommand;
  ArcadeDrive driveBaseArcadeDriveCommand;
  RunIntake runIntakeCommand;
  RunOuttake runOuttakeCommand;
  ShooterSetPower runShooterPower;
  ShooterPIDController runShooterPID;
  RunArmPosition runArmPosition;
  RunArmPower runArmPower;
  DecreaseArmSetPoint decreaseArmSetPoint;
  IncreaseArmSetPoint increaseArmSetPoint;

  private static final XboxController xboxController = new XboxController(RobotMap.JoyStickConstants.xboxControllerPort);
  private static Joystick leftJoystick = new Joystick(RobotMap.JoyStickConstants.leftJoystickPort);
  private static Joystick rightJoystick = new Joystick(RobotMap.JoyStickConstants.rightJoystickPort);
  private  JoystickButton driverLeftTrigger = new JoystickButton(leftJoystick,RobotMap.JoyStickConstants.leftJoystickTrigger );
  private JoystickButton driverRightTrigger = new JoystickButton(rightJoystick, RobotMap.JoyStickConstants.rightJoystickTrigger);
  private JoystickButton driverThreeButton = new JoystickButton(rightJoystick, RobotMap.JoyStickConstants.rightJoystickThreeButton);
  private JoystickButton driverFiveButton = new JoystickButton(rightJoystick, RobotMap.JoyStickConstants.rightJoystickFiveButton);


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
    CameraServer.startAutomaticCapture();
    driverBaseSubsystem = new frc.robot.subsystems.DriveTrain();
    //intakesubsystem = new frc.robot.subsystems.Intake();
    

    //shootersubsystem = new frc.robot.subsystems.Shooter();
    armsubsystem = new frc.robot.subsystems.Arms();
  }

  private void initCommands(){
    //driveBaseTankDriveCommand = new TankDrive(leftJoystick, rightJoystick, driverBaseSubsystem);
    //driveBaseArcadeDriveCommand = new ArcadeDrive(xboxController, driverBaseSubsystem);
    //runIntakeCommand = new RunIntake(intakesubsystem);
    //runOuttakeCommand = new RunOuttake(intakesubsystem);
    //driverBaseSubsystem.setDefaultCommand(driveBaseArcadeDriveCommand);
    //runShooterPower = new ShooterSetPower(shootersubsystem);
    //runShooterPID = new ShooterPIDController(shootersubsystem);
    runArmPosition = new RunArmPosition(armsubsystem);
    runArmPower = new RunArmPower(armsubsystem);
    decreaseArmSetPoint = new DecreaseArmSetPoint(armsubsystem);
    increaseArmSetPoint = new IncreaseArmSetPoint(armsubsystem);
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //driverLeftTrigger.whileTrue(runIntakeCommand);
    //driverRightTrigger.whileTrue(runOuttakeCommand);
   // driverRightTrigger.whileTrue(runArmPower);
    //driverLeftTrigger.whileTrue(runArmPosition);
    //driverBaseSubsystem.setDefaultCommand(driveBaseTankDriveCommand);
    driverRightTrigger.whileTrue(runArmPower);
    driverLeftTrigger.whileTrue(runArmPosition);

    driverThreeButton.onTrue(decreaseArmSetPoint);
    driverFiveButton.onTrue(increaseArmSetPoint);
    

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
  }
  
  private void configureShuffleBoard(){
    
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   */
  //public Command getAutonomousCommand() {
    // An example command will be run in autonomous

  }
//}
