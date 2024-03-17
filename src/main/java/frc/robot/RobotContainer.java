// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ArmSubsystemCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.SwerveGamepadDriveCommand;
import frc.robot.commands.IntakeSubsystemCommand;
import frc.robot.commands.ShooterSubsystemCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController commandXboxController = new CommandXboxController(
      OperatorConstants.DRIVE_TRAIN_XBOX_CONTROLLER_PORT);
   private final XboxController xboxController = new XboxController(
    OperatorConstants.ARM_AND_SHOOTAKE_XBOX_CONTROLLER_PORT);

  // The robot's subsystems and commands are defined here...
  private final DriveTrainSubsystem driveTrain = new DriveTrainSubsystem();
  private final IntakeSubsystem inTake = new IntakeSubsystem(IntakeConstants.INTAKE_CAN_ID, IntakeConstants.INTAKE_MOTOR_SPEED,false);
  private final ArmSubsystem Arm = new ArmSubsystem(ArmConstants.ARM_MOTOR_LEFT_CAN_ID, ArmConstants.ARM_MOTOR_RIGHT_CAN_ID, ArmConstants.ARM_MOTOR_SPEED, false);
  private final ShooterSubsystem Shooter = new ShooterSubsystem(ShooterConstants.SHOOTER_MOTOR_LEFT_CAN_ID, ShooterConstants.SHOOTER_MOTOR_RIGHT_CAN_ID, ShooterConstants.SHOOTER_MOTOR_SPEED, false);

  private final SendableChooser<Boolean> fieldRelativeChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    fieldRelativeChooser.setDefaultOption("Field Relative",  true);
    fieldRelativeChooser.addOption("Robot Relative", false);
    //SmartDashboard.putData(fieldRelativeChooser);
  
    driveTrain.setDefaultCommand(new SwerveGamepadDriveCommand(driveTrain,commandXboxController::getLeftX, commandXboxController::getLeftY, commandXboxController::getRightX, fieldRelativeChooser::getSelected));
    inTake.setDefaultCommand(new IntakeSubsystemCommand(inTake, xboxController::getAButton, xboxController::getLeftBumper, true));
    Shooter.setDefaultCommand(new ShooterSubsystemCommand(Shooter, Arm.armPosition, xboxController::getBButton, xboxController::getRightBumper, true));
    Arm.setDefaultCommand(new ArmSubsystemCommand(Arm, xboxController::getBackButton, xboxController::getStartButton, xboxController::getXButton, xboxController::getYButton, false));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    commandXboxController.rightBumper().whileTrue(driveTrain.run(driveTrain::setX));

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Autos.templateAuto(driveTrain);   
  }
}
