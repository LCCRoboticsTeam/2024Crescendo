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
//import com.pathplanner.lib.path.*;
//import com.pathplanner.lib.auto.*;
//import com.pathplanner.lib.commands.*;
//import com.pathplanner.lib.controllers.*;
//import com.pathplanner.lib.pathfinding.*;
//import com.pathplanner.lib.util.*;
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

        PathPlannerPath pathCenterSpeakerIn = PathPlannerPath.fromPathFile("CenterSpeakerIn");
        pathCenterSpeakerIn.preventFlipping=true;
        PathPlannerPath pathCenterSpeakerOut = PathPlannerPath.fromPathFile("CenterSpeakerOut");
        pathCenterSpeakerOut.preventFlipping=true;
        PathPlannerPath pathLeftSpeakerIn = PathPlannerPath.fromPathFile("LeftSpeakerIn");
        pathLeftSpeakerIn.preventFlipping=true;
        PathPlannerPath pathLeftSpeakerOut = PathPlannerPath.fromPathFile("LeftSpeakerOut");
        pathLeftSpeakerOut.preventFlipping=true;
        PathPlannerPath pathRightSpeakerIn = PathPlannerPath.fromPathFile("RightSpeakerIn");
        pathRightSpeakerIn.preventFlipping=true;
        PathPlannerPath pathRightSpeakerOut = PathPlannerPath.fromPathFile("RightSpeakerOut");
        pathRightSpeakerOut.preventFlipping=true;
        PathPlannerPath pathCenterLineIn = PathPlannerPath.fromPathFile("CenterlineIn");
        PathPlannerPath pathCenterLineOut = PathPlannerPath.fromPathFile("CenterlineOut");

        switch (autoType) {
            case ONE_NOTE_CENTER:
                // Reset odometry to the starting pose of the trajectory.
                driveTrain.resetOdometry(pathCenterSpeakerOut.getPreviewStartingHolonomicPose());
                addCommands(
                    // Zero the ARM encoder to Reverse Limit position
                    new ArmToReverseLimitCommand(Arm),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, 0),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    // Move ARM to Intake position
                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE),
                    // Move out of starting zone
                    // This is for trying auto instead of path which should reset the odometry automatically
                    //new PathPlannerAuto("CenterSpeakerOut")
                    AutoBuilder.followPath(pathCenterSpeakerOut)
                );
                break;
            case TWO_NOTE_CENTER:
                // Reset odometry to the starting pose of the trajectory.
                driveTrain.resetOdometry(pathCenterSpeakerOut.getPreviewStartingHolonomicPose());
                addCommands(
                    // Zero the ARM encoder to Reverse Limit position
                    new ArmToReverseLimitCommand(Arm),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, 0),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    // Move ARM to Intake position
                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE),
                    // Start the intake and move out of the starting zone, intake will stop once note is detected
                    new ParallelCommandGroup(new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, 0, false),
                                             AutoBuilder.followPath(pathCenterSpeakerOut)),
                    // Moving back in to Speaker position
                    AutoBuilder.followPath(pathCenterSpeakerIn),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, 0),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    AutoBuilder.followPath(pathCenterSpeakerOut)
                );
                break;
            case ONE_NOTE_RIGHT:
                // Reset odometry to the starting pose of the trajectory.
                driveTrain.resetOdometry(pathRightSpeakerOut.getPreviewStartingHolonomicPose());
                addCommands(
                    // Zero the ARM encoder to Reverse Limit position
                    new ArmToReverseLimitCommand(Arm),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, 0),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    // Move ARM to Intake position
                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE),
                    // Move out of starting zone
                    AutoBuilder.followPath(pathRightSpeakerOut)
                );
                break;
            case TWO_NOTE_RIGHT:
                // Reset odometry to the starting pose of the trajectory.
                driveTrain.resetOdometry(pathRightSpeakerOut.getPreviewStartingHolonomicPose());
                addCommands(
                    // Zero the ARM encoder to Reverse Limit position
                    new ArmToReverseLimitCommand(Arm),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, 0),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    // Move ARM to Intake position
                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE),
                    // Start the intake and move out of the starting zone, intake will stop once note is detected
                    new ParallelCommandGroup(new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, 0, false),
                                             AutoBuilder.followPath(pathRightSpeakerOut)),
                    // Moving back in to Speaker position
                    AutoBuilder.followPath(pathRightSpeakerIn),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, 0),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    AutoBuilder.followPath(pathRightSpeakerOut)                         
                );
                break;
            case ONE_NOTE_LEFT:
                // Reset odometry to the starting pose of the trajectory.
                driveTrain.resetOdometry(pathLeftSpeakerOut.getPreviewStartingHolonomicPose());
                addCommands(
                    // Zero the ARM encoder to Reverse Limit position
                    new ArmToReverseLimitCommand(Arm),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, 0),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    // Move ARM to Intake position
                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE),
                    // Move out of starting zone
                    AutoBuilder.followPath(pathLeftSpeakerOut)
                );
                break;
            case TWO_NOTE_LEFT:
                // Reset odometry to the starting pose of the trajectory.
                driveTrain.resetOdometry(pathLeftSpeakerOut.getPreviewStartingHolonomicPose());
                addCommands(
                    // Zero the ARM encoder to Reverse Limit position
                    new ArmToReverseLimitCommand(Arm),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, 0),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    // Move ARM to Intake position
                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE),
                    // Start the intake and move out of the starting zone, intake will stop once note is detected
                    new ParallelCommandGroup(new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, 0, false),
                                             AutoBuilder.followPath(pathLeftSpeakerOut)),
                    // Moving back in to Speaker position
                    AutoBuilder.followPath(pathLeftSpeakerIn),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, 0),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    AutoBuilder.followPath(pathLeftSpeakerOut)                         
                );
                break;
            case TWO_NOTE_CENTERLINE:
                // Reset odometry to the starting pose of the trajectory.
                driveTrain.resetOdometry(pathCenterLineOut.getPreviewStartingHolonomicPose());
                addCommands(
                    // Zero the ARM encoder to Reverse Limit position
                    new ArmToReverseLimitCommand(Arm),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, 0),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    // Move ARM to Intake position
                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE),
                    // Start the intake and move out of the starting zone, intake will stop once note is detected
                    new ParallelCommandGroup(new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, 0, false),
                                             AutoBuilder.followPath(pathCenterLineOut)),
                    // Moving back in to Speaker position
                    AutoBuilder.followPath(pathCenterLineIn),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, 0),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    AutoBuilder.followPath(pathCenterLineOut)                         
                );
                break;
            case MOVE_OUT: default:
                // Reset odometry to the starting pose of the trajectory.
                driveTrain.resetOdometry(pathCenterSpeakerOut.getPreviewStartingHolonomicPose());
                addCommands(
                // Move out of starting zone
                new ArmToReverseLimitCommand(Arm),
                AutoBuilder.followPath(pathCenterSpeakerOut)
                );
                break;
        }
    }
}

