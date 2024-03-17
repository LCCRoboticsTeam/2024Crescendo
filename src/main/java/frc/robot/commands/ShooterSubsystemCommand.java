// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterSubsystemCommand extends Command {

  private final ShooterSubsystem ShooterSubsystem;
  //private final XboxController xboxController;
  private final BooleanSupplier ShooterIn;
  private final BooleanSupplier ShooterOut;
  private final boolean ShooterOutOnly;

  private final ArmPosition armPosition;

  
  /** Creates a new SwerveControllerDrive. */
  public ShooterSubsystemCommand(ShooterSubsystem ShooterSubsystem, ArmPosition armPosition, BooleanSupplier ShooterOut, BooleanSupplier ShooterIn, boolean ShooterOutOnly) {
    this.ShooterSubsystem = ShooterSubsystem;
    this.armPosition = armPosition;
    this.ShooterOut = ShooterOut;
    this.ShooterIn = ShooterIn;
    this.ShooterOutOnly = ShooterOutOnly;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ShooterSubsystem.ShooterOff(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    if (ShooterOut.getAsBoolean()){
      if (armPosition==ArmPosition.SPEAKER_SHOOTER)
        ShooterSubsystem.ShooterOut(true);
      else
        ShooterSubsystem.ShooterOut(false);
    } 
    else {
      if ((!ShooterOutOnly) && (ShooterIn.getAsBoolean())) {
        ShooterSubsystem.ShooterIn();
      } 
      else {
        ShooterSubsystem.ShooterOff(); 
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterSubsystem.ShooterOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
