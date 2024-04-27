// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToPositionCommand extends Command {

  private ArmPosition targetPosition;

  private ArmSubsystem armSubsystem;

  public ArmToPositionCommand(ArmSubsystem armSubsystem, ArmPosition targetPosition) {
    this.armSubsystem = armSubsystem;
    this.targetPosition = targetPosition;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.setArmPosition(ArmPosition.MOVING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int distanceToGo = targetPosition.getPosition() - armSubsystem.getBoreEncoderVal();
    if (distanceToGo <= 0) {
      armSubsystem.moveArmUp();
    } else {
      armSubsystem.moveArmDown();
    }
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isAboveDeadzoneMin = armSubsystem.getBoreEncoderVal() >= (targetPosition.getPosition() - ArmConstants.ARM_MOVE_DEADZONE / 2);
    boolean isBelowDeadzoneMax = armSubsystem.getBoreEncoderVal() <= (targetPosition.getPosition() + ArmConstants.ARM_MOVE_DEADZONE / 2);
    return isAboveDeadzoneMin && isBelowDeadzoneMax;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.moveStop();
    if (!interrupted) {
      armSubsystem.setArmPosition(targetPosition);
      if (targetPosition==ArmPosition.SPEAKER_SHOOTER) {
        armSubsystem.holdArmUp();
      }
    }
  }

}
