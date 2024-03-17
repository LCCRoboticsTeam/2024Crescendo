// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmSubsystemCommand extends Command {

  private final ArmSubsystem ArmSubsystem;
  private final BooleanSupplier ReverseForwardPosition;
  private final BooleanSupplier UprightPosition;
  private final BooleanSupplier IntakeAndAmpPosition;
  private final BooleanSupplier SpeakerShooterPosition;
  
  public ArmSubsystemCommand(ArmSubsystem ArmSubsystem, BooleanSupplier ReverseForwardPosition, BooleanSupplier UprightPosition, BooleanSupplier IntakeAndAmpPosition, BooleanSupplier SpeakerShooterPosition, boolean printDebugInput) {
    this.ArmSubsystem = ArmSubsystem;
    this.ReverseForwardPosition = ReverseForwardPosition;
    this.UprightPosition = UprightPosition;
    this.IntakeAndAmpPosition = IntakeAndAmpPosition;
    this.SpeakerShooterPosition = SpeakerShooterPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.ArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // By default will do this incase there is scenario were we plan to start with 
    // ARM in known 0 position.
    ArmSubsystem.InitializeEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    if (ReverseForwardPosition.getAsBoolean()) {
      if (ArmSubsystem.armPosition != ArmPosition.REVERSE_LIMIT){
        ArmSubsystem.ReverseLimitPosition(true);
      }
      else {
        ArmSubsystem.ForwardLimitPosition();
      } 
    }
    if (UprightPosition.getAsBoolean() && (ArmSubsystem.armPosition != ArmPosition.UPRIGHT)) {
      ArmSubsystem.UprightPosition();
      return;
    }
    if (IntakeAndAmpPosition.getAsBoolean()){
      if (ArmSubsystem.armPosition != ArmPosition.INTAKE){
        ArmSubsystem.IntakePosition();
      } 
      else {
        ArmSubsystem.AmpShooterPosition();
      }
      return;
    }
    if (SpeakerShooterPosition.getAsBoolean() && (ArmSubsystem.armPosition != ArmPosition.SPEAKER_SHOOTER)) {
      ArmSubsystem.SpeakerShooterPosition();
      return;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //ArmSubsystem.UprightPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
