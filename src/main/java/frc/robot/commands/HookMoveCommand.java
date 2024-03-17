// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HookSubsystem;

public class HookMoveCommand extends Command {

  public enum Direction {
    UP,
    DOWN
  }

  private HookSubsystem hookSubsystem;
  private Direction direction;

  /** Creates a new HookMoveCommand. */
  public HookMoveCommand(HookSubsystem hookSubsystem, Direction direction) {
    this.hookSubsystem = hookSubsystem;
    this.direction = direction;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hookSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (direction.equals(Direction.UP)) {
      hookSubsystem.moveUp();
    } else {
      hookSubsystem.moveDown();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hookSubsystem.moveStop();
    if (!interrupted) {
      hookSubsystem.setSolenoidState(false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hookSubsystem.limitSwitchClosed();
  }
}
