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
import frc.robot.commands.IntakeMoveOutCommand;
import frc.robot.commands.SwerveGamepadDriveCommand;
import frc.robot.commands.HookMoveCommand.Direction;
import frc.robot.commands.ShooterMoveOutCommand;
import frc.robot.commands.ShooterMoveInCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.HookSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LEDController;

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
  private final IntakeSubsystem inTake = new IntakeSubsystem(IntakeConstants.INTAKE_MOTOR_CAN_ID, 
                                                             IntakeConstants.INTAKE_LASERCAN_0_CAN_ID, 
                                                             IntakeConstants.INTAKE_LASERCAN_1_CAN_ID,
                                                             IntakeConstants.INTAKE_MOTOR_SPEED);
  private final ArmSubsystem Arm = new ArmSubsystem(ArmConstants.ARM_MOTOR_LEFT_CAN_ID,
                                                    ArmConstants.ARM_MOTOR_RIGHT_CAN_ID, 
                                                    ArmConstants.ARM_MOTOR_SPEED_UP, 
                                                    ArmConstants.ARM_MOTOR_SPEED_DOWN);
  private final ShooterSubsystem Shooter = new ShooterSubsystem(ShooterConstants.SHOOTER_MOTOR_LEFT_CAN_ID,
                                                                ShooterConstants.SHOOTER_MOTOR_RIGHT_CAN_ID, 
                                                                ShooterConstants.SHOOTER_MOTOR_SPEED);
  private final HookSubsystem hookSubsystem = new HookSubsystem(HOOK_MOTOR_CAN_ID, 
                                                                HOOK_SOLENOID_CAN_ID);
  private final LEDController ledController = new LEDController();

  private final SendableChooser<Boolean> fieldRelativeChooser = new SendableChooser<>();

  private final SendableChooser<Double> fieldRelativeChooser_IntakeMoveInDelay = new SendableChooser<>();

  private final double[] addToDelay = {0, 100, 200, 300, 400, 500};

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    fieldRelativeChooser.setDefaultOption("Field Relative", true);
    fieldRelativeChooser.addOption("Robot Relative", false);

    //fieldRelativeChooser_IntakeMoveInDelay.setDefaultOption("0", addToDelay[0]);
    //for (int i = 1; i < addToDelay.length; i++) {
    //  fieldRelativeChooser_IntakeMoveInDelay.addOption(String.valueOf(addToDelay[i]), addToDelay[i]);
    //}
    
    SmartDashboard.putData(fieldRelativeChooser);
    SmartDashboard.putData(fieldRelativeChooser_IntakeMoveInDelay);
    SmartDashboard.putNumber("Arm Encoder Value", Arm.getBoreEncoderVal());

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

    //  ARM
    ////////////////////////////////////////////////
    //    ACTIVE default (Safety=OFF, Alternate=OFF)
    //     ^ ARM AMP Position
    //     v ARM INTAKE Position
    commandLaunchpad.safety().negate().and(commandLaunchpad.armUp()
                                      .and(commandLaunchpad.miscBlue().negate()))
                                      .onTrue(new ArmToPositionCommand(Arm, ArmPosition.AMP_SHOOTER));
    commandLaunchpad.safety().negate().and(commandLaunchpad.armDown()
                                      .and(commandLaunchpad.miscBlue().negate()))
                                      .onTrue(new ArmToPositionCommand(Arm, ArmPosition.INTAKE));
    //   ACTIVE Alternate (Safety=OFF, Alternate=ON)
    //     ^ ARM SHOOTER Position
    //     v ARM Reverse Limit Position (Note already happens by default at Teleopt start)
    commandLaunchpad.safety().negate().and(commandLaunchpad.armUp()
                                      .and(commandLaunchpad.miscBlue()))
                                      .onTrue(new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER));
    commandLaunchpad.safety().negate().and(commandLaunchpad.armDown()
                                      .and(commandLaunchpad.miscBlue()))
                                      .onTrue(new ArmToReverseLimitCommand(Arm));
    //   ACTIVE Safety ON (Safety=OFF, Alternate=N/A)
    //     ^ ARM Manual UP
    //     v ARM Manual DOWN
    commandLaunchpad.safety().and(commandLaunchpad.armUp()).whileTrue(new ArmMoveCommand(Arm, true, hookSubsystem));
    commandLaunchpad.safety().and(commandLaunchpad.armDown()).whileTrue(new ArmMoveCommand(Arm, false, hookSubsystem));
    ////////////////////////////////////////////////

    //  SHOOTAKE
    ////////////////////////////////////////////////
    //    ACTIVE default (Safety=N/A, Alternate=OFF)
    //      ^ INTAKE IN
    //      v SHOOTER OUT
    //commandLaunchpad.intakeIn().and(commandLaunchpad.miscBlue().negate()).whileTrue(new IntakeMoveInCommand(inTake));
    commandLaunchpad.intakeIn().and(commandLaunchpad.miscBlue().negate())
                               .onTrue(new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, 0, false));
    commandLaunchpad.shooterOut().and(commandLaunchpad.miscBlue().negate())
                                 .whileTrue(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, ShooterConstants.SHOOTER_MOVE_OUT_DELAY_IN_MS))
                                 .onTrue(new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true));
    ////////////////////////////////////////////////
    //    ACTIVE default (Safety=N/A, Alternate=ON)
    //      ^ INTAKE OUT (Normally never needed)
    //      v SHOOTER IN (Normally never needed)
    commandLaunchpad.intakeIn().and(commandLaunchpad.miscBlue()).whileTrue(new IntakeMoveOutCommand(inTake));
    commandLaunchpad.shooterOut().and(commandLaunchpad.miscBlue()).whileTrue(new ShooterMoveInCommand(Shooter));

    commandLaunchpad.safety().onTrue(new ArmToPositionCommand(Arm, ArmPosition.HANG));

    commandLaunchpad.safety().and(commandLaunchpad.climbUp())
      .whileTrue(new HookMoveCommand(hookSubsystem, Direction.UP));
    commandLaunchpad.safety().and(commandLaunchpad.climbDown())
      .whileTrue(new HookMoveCommand(hookSubsystem, Direction.DOWN))
      .whileTrue(new ArmMoveCommand(Arm, false, hookSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Autos.templateAuto(driveTrain);
    //return null;  
  }

  public Command getTeleopInitCommand() {
    return new ArmToReverseLimitCommand(Arm);
    //return null;  
  }

}
