package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
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

public class ComplexAuto extends SequentialCommandGroup {
    
    public ComplexAuto(DriveTrainSubsystem driveTrain, ArmSubsystem Arm, IntakeSubsystem inTake, ShooterSubsystem Shooter, LEDController ledController, AutoTypes autoType) {

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.DRIVE_KINEMATICS);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            //List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            List.of(new Translation2d(1, 0), new Translation2d(1.5, 0)),
            // End 2 meters straight ahead of where we started, facing forward
            new Pose2d(2, 0, new Rotation2d(0)),
            config);

        Trajectory exampleTrajectoryReverse = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            //List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            List.of(new Translation2d(-1, 0), new Translation2d(-1.5, 0)),
            // End 2 meters straight ahead of where we started, facing forward
            new Pose2d(-2, 0, new Rotation2d(0)),
            config);

        var thetaController = new ProfiledPIDController(
            AutoConstants.P_THETA_CONTROLLER, 0, 0, AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            exampleTrajectory,
            driveTrain::getPose, // Functional interface to feed supplier
            DriveConstants.DRIVE_KINEMATICS,
            // Position controllers
            new PIDController(AutoConstants.P_X_CONTROLLER, 0, 0),
            new PIDController(AutoConstants.P_Y_CONTROLLER, 0, 0),
            thetaController,
            driveTrain::setModuleStates,
            driveTrain);

        SwerveControllerCommand swerveControllerCommandReverse = new SwerveControllerCommand(
            exampleTrajectoryReverse,
            driveTrain::getPose, // Functional interface to feed supplier
            DriveConstants.DRIVE_KINEMATICS,
            // Position controllers
            new PIDController(AutoConstants.P_X_CONTROLLER, 0, 0),
            new PIDController(AutoConstants.P_Y_CONTROLLER, 0, 0),
            thetaController,
            driveTrain::setModuleStates,
            driveTrain);

        // Reset odometry to the starting pose of the trajectory.
        driveTrain.resetOdometry(exampleTrajectory.getInitialPose());

        switch (autoType) {
            case ONE_NOTE:
                addCommands(
                    new ArmToReverseLimitCommand(Arm),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, ShooterConstants.SHOOTER_MOVE_OUT_DELAY_IN_MS),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    // Move ARM to Intake position
                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE),
                    // Move out of starting zone
                    swerveControllerCommand.andThen(() -> driveTrain.drive(0, 0, 0, true, true))
                );
                break;
            case TWO_NOTE_CENTER:
                addCommands(
                    new ArmToReverseLimitCommand(Arm),
                    // Move ARM to Speaker position
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER),
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, ShooterConstants.SHOOTER_MOVE_OUT_DELAY_IN_MS),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true)),
                    // Move ARM to Intake position
                    new ArmToPositionCommand(Arm, ArmPosition.INTAKE),
                    // Move out of starting zone and start Intake
                    
                   new ParallelCommandGroup(new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, 0, false),
                    swerveControllerCommand.andThen(() -> driveTrain.drive(0, 0, 0, true, true))),
                    // Moving back in and ARM to Speaker position
                    new ParallelCommandGroup(swerveControllerCommandReverse.andThen(() -> driveTrain.drive(0, 0, 0, true, true)),
                    new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER)),
                    
                    // Shoot
                    new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, ShooterConstants.SHOOTER_MOVE_OUT_DELAY_IN_MS),
                                             new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true))
                    //swerveControllerCommand.andThen(() -> driveTrain.drive(0, 0, 0, true, true))                         
                );
                break;
            case MOVE_OUT: default:
                addCommands(
                // Move out of starting zone
                new ArmToReverseLimitCommand(Arm),
                swerveControllerCommand.andThen(() -> driveTrain.drive(0, 0, 0, true, true))
                );
                break;
        }
    }
}

