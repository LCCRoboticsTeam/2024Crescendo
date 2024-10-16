package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoTypes;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LEDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

public class ComplexAuto extends SequentialCommandGroup {
    
    public ComplexAuto(DriveTrainSubsystem driveTrain, ArmSubsystem Arm, IntakeSubsystem inTake, ShooterSubsystem Shooter, LEDController ledController, XboxController xboxController, AutoTypes autoType) {

        var thetaController = new ProfiledPIDController(
            AutoConstants.P_THETA_CONTROLLER, 0, 0, AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI); 

        switch (autoType) {
            case ONE_NOTE_CENTER:
                addCommands(
                    // Zero the ARM encoder to Reverse Limit position
                    new ArmToReverseLimitCommand(Arm),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, ShooterConstants.SHOOTER_MOVE_OUT_DELAY_IN_MS),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    // Move ARM to Intake position
                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE),
                    // Move out of starting zone
                    new PathPlannerAuto("CenterSpeakerOut"),

                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE)
                );
                break;
            case TWO_NOTE_CENTER:
                addCommands(
                    // Zero the ARM encoder to Reverse Limit position
                    new ArmToReverseLimitCommand(Arm),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, ShooterConstants.SHOOTER_MOVE_OUT_DELAY_IN_MS),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    // Move ARM to Intake position
                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE),
                    // Start the intake and move out of the starting zone, intake will stop once note is detected
                    new ParallelCommandGroup(new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, 0, false),
                                             new PathPlannerAuto("CenterSpeakerOut")),
                    // Moving back in to Speaker position
                    new PathPlannerAuto("CenterSpeakerIn"),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, ShooterConstants.SHOOTER_MOVE_OUT_DELAY_IN_MS),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    new PathPlannerAuto("CenterSpeakerOut"),

                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE))

                ;
                
                break;
            case ONE_NOTE_RIGHT:
                addCommands(
                    // Zero the ARM encoder to Reverse Limit position
                    new ArmToReverseLimitCommand(Arm),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, ShooterConstants.SHOOTER_MOVE_OUT_DELAY_IN_MS),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    // Move ARM to Intake position
                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE),
                    // Move out of starting zone
                    new PathPlannerAuto("RightSpeakerOut"),

                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE)
                );
                break;
            case TWO_NOTE_RIGHT:
                addCommands(
                    // Zero the ARM encoder to Reverse Limit position
                    new ArmToReverseLimitCommand(Arm),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, ShooterConstants.SHOOTER_MOVE_OUT_DELAY_IN_MS),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    // Move ARM to Intake position
                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE),
                    // Start the intake and move out of the starting zone, intake will stop once note is detected
                    new ParallelCommandGroup(new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, 0, false),
                                             new PathPlannerAuto("RightSpeakerOut")),
                    // Moving back in to Speaker position
                    new PathPlannerAuto("RightSpeakerIn"),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, ShooterConstants.SHOOTER_MOVE_OUT_DELAY_IN_MS),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    new PathPlannerAuto("RightSpeakerOut"),
                    
                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE)
                );
                break;
            case ONE_NOTE_LEFT:
                addCommands(
                    // Zero the ARM encoder to Reverse Limit position
                    new ArmToReverseLimitCommand(Arm),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, ShooterConstants.SHOOTER_MOVE_OUT_DELAY_IN_MS),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    // Move ARM to Intake position
                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE),
                    // Move out of starting zone
                    new PathPlannerAuto("LeftSpeakerOut"),

                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE)
                );
                break;
            case TWO_NOTE_LEFT:
                addCommands(
                    // Zero the ARM encoder to Reverse Limit position
                    new ArmToReverseLimitCommand(Arm),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, ShooterConstants.SHOOTER_MOVE_OUT_DELAY_IN_MS),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    // Move ARM to Intake position
                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE),
                    // Start the intake and move out of the starting zone, intake will stop once note is detected
                    new ParallelCommandGroup(new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, 0, false),
                                             new PathPlannerAuto("LeftSpeakerOut")),
                    // Moving back in to Speaker position
                    new PathPlannerAuto("LeftSpeakerIn"),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, ShooterConstants.SHOOTER_MOVE_OUT_DELAY_IN_MS),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    new PathPlannerAuto("LeftSpeakerOut"),
                    
                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE)
                );
                break;
            case TWO_NOTE_CENTERLINE:
                addCommands(
                    // Zero the ARM encoder to Reverse Limit position
                    new ArmToReverseLimitCommand(Arm),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, ShooterConstants.SHOOTER_MOVE_OUT_DELAY_IN_MS),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    // Move ARM to Intake position
                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE),
                    // Start the intake and move out of the starting zone, intake will stop once note is detected
                    new ParallelCommandGroup(new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, 0, false),
                                             new PathPlannerAuto("CenterlineOut")),
                    // Moving back in to Speaker position
                    new PathPlannerAuto("CenterlineIn"),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, ShooterConstants.SHOOTER_MOVE_OUT_DELAY_IN_MS),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    new PathPlannerAuto("CenterlineOut"), 
                    
                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE)
                );
                break;
            case FOUR_NOTE_CENTER:
                addCommands(
                    // Zero the ARM encoder to Reverse Limit position
                    new ArmToReverseLimitCommand(Arm),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, ShooterConstants.SHOOTER_MOVE_OUT_DELAY_IN_MS),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    // Move ARM to Intake position
                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE),
                    // Start the intake and move out of the starting zone, intake will stop once note is detected
                    new ParallelCommandGroup(new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, 0, false),
                                             new PathPlannerAuto("CenterSpeakerOut")),
                    // Moving back in to Speaker position
                    new PathPlannerAuto("CenterSpeakerIn"),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, ShooterConstants.SHOOTER_MOVE_OUT_DELAY_IN_MS),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),

                    // Move ARM to Intake position
                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE),
                    // Start the intake and move out of the starting zone, intake will stop once note is detected
                    new ParallelCommandGroup(new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, 0, false),
                                             new PathPlannerAuto("CenterOutSTP1")),
                    // Moving back in to Speaker position
                    new PathPlannerAuto("CenterInSTP1"),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, ShooterConstants.SHOOTER_MOVE_OUT_DELAY_IN_MS),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    
                    // Move ARM to Intake position
                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE),
                    // Start the intake and move out of the starting zone, intake will stop once note is detected
                    new ParallelCommandGroup(new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, 0, false),
                                             new PathPlannerAuto("CenterOutSTP2")),
                    // Moving back in to Speaker position
                    new PathPlannerAuto("CenterInSTP2"),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, ShooterConstants.SHOOTER_MOVE_OUT_DELAY_IN_MS),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    

                    new PathPlannerAuto("CenterSpeakerOut"),


                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE))

                ;
                
                break;
            case MOVE_OUT: default:
                addCommands(
                // Move out of starting zone
                new ArmToReverseLimitCommand(Arm),
                new PathPlannerAuto("CenterSpeakerOut")
                );
                break;
        }
    }
}

