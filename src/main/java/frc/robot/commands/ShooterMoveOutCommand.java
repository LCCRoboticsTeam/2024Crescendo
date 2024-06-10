// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.LEDColorState;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LEDController;

public class ShooterMoveOutCommand extends Command {

  private final ShooterSubsystem shooterSubsystem;
  private final Supplier<ArmPosition> armPosition;
  private final LEDController ledController;
  private final XboxController xboxController;
  private Boolean RumbleToggle;
  private int isFinishedDelayCount = 0;
  private int isFinishedDelayInMs = 0;

  /** Creates a new IntakeMoveCommand. */
  public ShooterMoveOutCommand(ShooterSubsystem shooterSubsystem, Supplier<ArmPosition> armPosition, LEDController ledController, XboxController xboxController, int isFinishedDelayInMs) {
    this.shooterSubsystem = shooterSubsystem;
    this.armPosition = armPosition;
    this.ledController = ledController;
    this.xboxController = xboxController;
    this.isFinishedDelayInMs = isFinishedDelayInMs;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  isFinishedDelayCount=0;
  RumbleToggle=true;
  //xboxController.setRumble(RumbleType.kBothRumble, 0.0);
  
  ledController.setColor(LEDColorState.SHOOTING);
  if (armPosition.get().equals(ArmPosition.AMP_SHOOTER)) {
    isFinishedDelayInMs=(isFinishedDelayInMs/ShooterConstants.SHOOTER_MOVE_OUT_DELAY_ARM_POSITION_AMP_SHOOTER_DIVIDER);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.ShooterOut(armPosition.get().equals(ArmPosition.SPEAKER_SHOOTER) || armPosition.get().equals(ArmPosition.INTAKE));
    
    //if (RumbleToggle) {
    //    xboxController.setRumble(RumbleType.kLeftRumble, 1.0);
    //    xboxController.setRumble(RumbleType.kRightRumble, 0.0);
    //}
    //else {
    //  xboxController.setRumble(RumbleType.kLeftRumble, 0.0);
    //  xboxController.setRumble(RumbleType.kRightRumble, 1.0);
    //}
    //RumbleToggle=!RumbleToggle;

    // This should be run every 20ms
    isFinishedDelayCount+=ShooterConstants.SHOOTER_EXECUTE_COUNT_INCREMENT_IN_MS;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.ShooterOff();
    xboxController.setRumble(RumbleType.kBothRumble, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // isFinishedDelayInMs=0 means will never stop command.
    if ((isFinishedDelayInMs!=0) && (isFinishedDelayCount>=isFinishedDelayInMs)) {
      xboxController.setRumble(RumbleType.kBothRumble, 0.0);
      return true;
    }
    else {
      return false;
    }
  }
}
