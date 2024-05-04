// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeMoveInCommand extends Command {

  private final IntakeSubsystem intakeSubsystem;
  private final Supplier<ArmPosition> armPosition;
  private boolean noteDetectedInitial;
  private int executeCount;
  private int executeDelayInMs;

  /** Creates a new IntakeMoveCommand. */
  public IntakeMoveInCommand(IntakeSubsystem intakeSubsystem, Supplier<ArmPosition> armPosition, int executeDelayInMs) {
    this.intakeSubsystem = intakeSubsystem;
    this.armPosition = armPosition;
    this.executeDelayInMs = executeDelayInMs;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Capture if the intake starts with a note or not
    this.noteDetectedInitial=intakeSubsystem.noteDetected();
    executeCount=0;
    if (armPosition.get().equals(ArmPosition.AMP_SHOOTER)) {
      executeDelayInMs=(executeDelayInMs/2);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (executeCount>=executeDelayInMs) {
      intakeSubsystem.intakeIn();
    } else {
      // This should be run every 20ms
      executeCount+=20;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.intakeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Only return true on intake of note or note is shot
    if (this.noteDetectedInitial!=intakeSubsystem.noteDetected())
      return true;
    else
      return false;
  }
}
