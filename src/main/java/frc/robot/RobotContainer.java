// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ArmMoveCommand;
import frc.robot.commands.ArmToForwardLimitCommand;
import frc.robot.commands.ArmToPositionCommand;
import frc.robot.commands.ArmToReverseLimitCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.HookMoveCommand;
import frc.robot.commands.IntakeMoveInCommand;
import frc.robot.commands.SwerveGamepadDriveCommand;
import frc.robot.commands.HookMoveCommand.Direction;
import frc.robot.commands.ShooterMoveOutCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.HookSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.HookConstants.HOOK_MOTOR_CAN_ID;
import static frc.robot.Constants.HookConstants.HOOK_SOLENOID_CAN_ID;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPosition;
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
  private final CommandXboxController commandXboxController = new CommandXboxController(OperatorConstants.XBOX_CONTROLLER_PORT);
  private final CommandLaunchpadController commandLaunchpad = new CommandLaunchpadController(OperatorConstants.LAUNCHPAD_PORT);

  // The robot's subsystems and commands are defined here...
  private final DriveTrainSubsystem driveTrain = new DriveTrainSubsystem();
  private final IntakeSubsystem inTake = new IntakeSubsystem(IntakeConstants.INTAKE_CAN_ID,
      IntakeConstants.INTAKE_MOTOR_SPEED);
  private final ArmSubsystem Arm = new ArmSubsystem(ArmConstants.ARM_MOTOR_LEFT_CAN_ID,
      ArmConstants.ARM_MOTOR_RIGHT_CAN_ID, ArmConstants.ARM_MOTOR_SPEED_UP, ArmConstants.ARM_MOTOR_SPEED_DOWN);
  private final ShooterSubsystem Shooter = new ShooterSubsystem(ShooterConstants.SHOOTER_MOTOR_LEFT_CAN_ID,
      ShooterConstants.SHOOTER_MOTOR_RIGHT_CAN_ID, ShooterConstants.SHOOTER_MOTOR_SPEED);
  private final HookSubsystem hookSubsystem = new HookSubsystem(HOOK_MOTOR_CAN_ID, HOOK_SOLENOID_CAN_ID);

  private final SendableChooser<Boolean> fieldRelativeChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    fieldRelativeChooser.setDefaultOption("Field Relative", true);
    fieldRelativeChooser.addOption("Robot Relative", false);
    
    // SmartDashboard.putData(fieldRelativeChooser);

    driveTrain.setDefaultCommand(new SwerveGamepadDriveCommand(driveTrain, commandXboxController::getLeftX,
        commandXboxController::getLeftY, commandXboxController::getRightX, fieldRelativeChooser::getSelected));
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
    //SmartDashboard.putBoolean("Reverse Limit Switch Closed", ArmSubsystem.revLimSwitchClosed);

    commandLaunchpad.safety().negate().and(commandLaunchpad.armUp().and(commandLaunchpad.miscBlue().negate())).onTrue(new ArmToPositionCommand(Arm, ArmPosition.AMP_SHOOTER));
    commandLaunchpad.safety().negate().and(commandLaunchpad.armUp().and(commandLaunchpad.miscBlue())).onTrue(new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER));

    commandLaunchpad.safety().negate().and(commandLaunchpad.armDown().and(commandLaunchpad.miscBlue().negate())).onTrue(new ArmToPositionCommand(Arm, ArmPosition.INTAKE));
    commandLaunchpad.safety().negate().and(commandLaunchpad.armDown().and(commandLaunchpad.miscBlue())).onTrue(new ArmToReverseLimitCommand(Arm));

    commandLaunchpad.safety().and(commandLaunchpad.armUp()).whileTrue(new ArmMoveCommand(Arm, true));

    commandLaunchpad.safety().and(commandLaunchpad.armDown()).whileTrue(new ArmMoveCommand(Arm, false));

    commandLaunchpad.intakeIn().whileTrue(new IntakeMoveInCommand(inTake));
    commandLaunchpad.shooterOut().whileTrue(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition));

    //commandLaunchpad.safety().onTrue(new ArmToPositionCommand(Arm, ArmPosition.HANG));

    commandLaunchpad.safety().and(commandLaunchpad.climbUp())
      .whileTrue(new HookMoveCommand(hookSubsystem, Direction.UP));
    commandLaunchpad.safety().and(commandLaunchpad.climbDown())
      .whileTrue(new HookMoveCommand(hookSubsystem, Direction.DOWN))
      .whileTrue(new ArmMoveCommand(Arm, false));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Autos.templateAuto(driveTrain);
  }

  public Command getTeleopInitCommand() {
    return new ArmToReverseLimitCommand(Arm);
    //return null;  
  }

}
