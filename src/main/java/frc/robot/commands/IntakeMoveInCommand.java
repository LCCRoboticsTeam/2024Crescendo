// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeMoveInCommand extends Command {

  private final IntakeSubsystem intakeSubsystem;
  private final Supplier<ArmPosition> armPosition;
  private int executeCount = 0;
  private int executeDelayInMs = 0;
  private Boolean high_speed;
  private int noteDetectedTrueCount;
  private int noteDetectedFalseCount;
  private Boolean executeDelayComplete;

  /** Creates a new IntakeMoveCommand. */
  public IntakeMoveInCommand(IntakeSubsystem intakeSubsystem, Supplier<ArmPosition> armPosition, int executeDelayInMs, Boolean high_speed) {
    this.intakeSubsystem = intakeSubsystem;
    this.armPosition = armPosition;
    this.executeDelayInMs = executeDelayInMs;
    this.high_speed = high_speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    executeCount=0;
    noteDetectedTrueCount=0;
    noteDetectedFalseCount=0;
    executeDelayComplete=false;
    if (armPosition.get().equals(ArmPosition.AMP_SHOOTER)) {
      executeDelayInMs=(executeDelayInMs/IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_ARM_POSITION_AMP_SHOOTER_DIVIDER);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (!executeDelayComplete && (executeCount>=executeDelayInMs)) {
      intakeSubsystem.intakeIn(high_speed);
      executeDelayComplete=true;
      System.out.println("The executeCount is " + executeCount);
    } else {
      // This should be run every 20ms
      executeCount+=IntakeConstants.INTAKE_EXECUTE_COUNT_INCREMENT_IN_MS;
      //System.out.println(executeCount);
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
    // Only return true on intake if note Detected or note is shot
    // Assumption is if executeDelayInMs=0, then we run until we detect a note, else 
    // we have a note and will run until we do not have a note (ie: it is shot)
    if (executeDelayInMs==0) {
      // Only the first detection starts counter
      if (intakeSubsystem.noteDetected() || noteDetectedTrueCount>0) {
        noteDetectedTrueCount++;
        //System.out.println("The noteDetectedTrueCount is " + noteDetectedTrueCount);
      }
      if (noteDetectedTrueCount>IntakeConstants.INTAKE_NOTE_DETECTED_TRUE_COUNT_THRESHOLD) {
        System.out.println("The noteDetectedTrueCount is " + noteDetectedTrueCount);
        return true;
      }
      else {
        return false;
      }
    }
    else {
      if (executeDelayComplete) {
        // Only the first detection starts counter
        if (!intakeSubsystem.noteDetected() || noteDetectedFalseCount>0) {
          noteDetectedFalseCount++;
          //System.out.println("The noteDetectedFalseCount is " + noteDetectedFalseCount);
          }
        if (noteDetectedFalseCount>IntakeConstants.INTAKE_NOTE_DETECTED_FALSE_COUNT_THRESHOLD) {
          System.out.println("The noteDetectedFalseCount is " + noteDetectedFalseCount);
          return true;
          }
        else {
          return false;
          }
        }
      else {
          return false;
        }
      }
    }
  }

//}