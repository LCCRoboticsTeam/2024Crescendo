// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterMoveOutCommand extends Command {

  private final ShooterSubsystem shooterSubsystem;
  private final Supplier<ArmPosition> armPosition;

  /** Creates a new IntakeMoveCommand. */
  public ShooterMoveOutCommand(ShooterSubsystem intakeSubsystem, Supplier<ArmPosition> armPosition) {
    this.shooterSubsystem = intakeSubsystem;
    this.armPosition = armPosition;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.ShooterOut(armPosition.get().equals(ArmPosition.SPEAKER_SHOOTER));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.ShooterOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
