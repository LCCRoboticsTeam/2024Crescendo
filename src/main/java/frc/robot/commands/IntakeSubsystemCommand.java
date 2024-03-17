// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeSubsystemCommand extends Command {

  private final IntakeSubsystem IntakeSubsystem;
  private final BooleanSupplier IntakeIn;
  private final BooleanSupplier IntakeOut;
  private final boolean IntakeInOnly;
  
  public IntakeSubsystemCommand(IntakeSubsystem IntakeSubsystem, BooleanSupplier IntakeIn, BooleanSupplier IntakeOut, boolean IntakeInOnly) {
    this.IntakeSubsystem = IntakeSubsystem;
    this.IntakeIn = IntakeIn;
    this.IntakeOut = IntakeOut;
    this.IntakeInOnly = IntakeInOnly;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IntakeSubsystem.intakeOff(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    if (IntakeIn.getAsBoolean()){
      IntakeSubsystem.intakeIn();
    } 
    else {
      if ((!IntakeInOnly) && (IntakeOut.getAsBoolean())) {
        IntakeSubsystem.intakeOut();
        }
      else {
        IntakeSubsystem.intakeOff();
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    IntakeSubsystem.intakeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
