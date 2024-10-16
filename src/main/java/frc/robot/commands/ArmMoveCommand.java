// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HookSubsystem;  

public class ArmMoveCommand extends Command {

  private ArmSubsystem armSubsystem;
  private boolean moveUp;
  private HookSubsystem hookSubsystem;

  /** Creates a new ArmMoveCommand. */
  public ArmMoveCommand(ArmSubsystem armSubsystem, boolean moveUp, HookSubsystem hookSubsystem) {
    this.armSubsystem = armSubsystem;
    this.moveUp = moveUp;
    this.hookSubsystem = hookSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem, hookSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.setArmPosition(ArmPosition.MOVING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (moveUp) {
      armSubsystem.moveArmUp();
    } else {
      armSubsystem.moveArmDown();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   //if ((!interrupted) || (!moveUp)) {
     // hookSubsystem.setSolenoidState(false);
      //armSubsystem.moveStop();
     // armSubsystem.setArmPosition(ArmPosition.FORWARD_LIMIT);
    //}else{
      hookSubsystem.setSolenoidState(false);
      armSubsystem.moveStop();
   // }
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   // if (!moveUp) {
     // return armSubsystem.isForwardLimitSwitchClosed();
   // }
   // else {
     return false;
   // }
    
  }
}
