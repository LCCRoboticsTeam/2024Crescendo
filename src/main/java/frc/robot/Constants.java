// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int XBOX_CONTROLLER_PORT = 0;
    public static final int DRIVE_TRAIN_XBOX_CONTROLLER_PORT = 0;
    public static final int ARM_AND_SHOOTAKE_XBOX_CONTROLLER_PORT = 0;
    public static final double XBOX_DEADBAND = 0.05;
  }

  public static final class AutoConstants {
    public static final double MAX_SPEED_METERS_PER_SECOND = 3;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

    public static final double P_X_CONTROLLER = 1;
    public static final double P_Y_CONTROLLER = 1;
    public static final double P_THETA_CONTROLLER = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
  }

public static final class IntakeConstants {
  public static final double INTAKE_MOTOR_SPEED = 0.4;
  public static final int INTAKE_BEAM_BREAK_DIO = 2;
  public static final int INTAKE_CAN_ID = 4;
}

public static final class ShooterConstants {
  public static final double SHOOTER_MOTOR_SPEED = 0.3;
  public static final double SHOOTER_HIGH_SPEED_MULTIPLIER = 2;
  public static final int SHOOTER_MOTOR_LEFT_CAN_ID = 5; 
  public static final int SHOOTER_MOTOR_RIGHT_CAN_ID = 9; 
}

public static final class ArmConstants {
  public static final double ARM_MOTOR_SPEED = 0.2;
  public static final int ARM_MOTOR_LEFT_CAN_ID = 7; 
  public static final int ARM_MOTOR_RIGHT_CAN_ID = 6; 
  public static final int ARM_BORE_ENCODER_CHANNEL_A_DIO = 0;
  public static final int ARM_BORE_ENCODER_CHANNEL_B_DIO = 1;
  public static final int ARM_UPRIGHT_BORE_ENCODER_POSITION = 250;
  public static final int ARM_AMP_SHOOTER_BORE_ENCODER_POSITION = 50;
  public static final int ARM_SPEAKER_SHOOTER_BORE_ENCODER_POSITION = 1000;
  public static final int ARM_HANG_BORE_ENCODER_POSITION = 400;
  public static final int ARM_INTAKE_BORE_ENCODER_POSITION = 900;
}

public enum ArmPosition {
  UNKNOWN, REVERSE_LIMIT, FORWARD_LIMIT, UPRIGHT, INTAKE, SPEAKER_SHOOTER, AMP_SHOOTER, HANG;
}

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double MAX_SPEED_METERS_PER_SECOND = 2;
    public static final double MAX_ANGULAR_SPEED =  Math.PI; // radians per second

    public static final double DIRECTION_SLEW_RATE = 0.6; // radians per second
    public static final double MAGNITUDE_SLEW_RATE = 0.9; // percent per second (1 = 100%)
    public static final double ROTATIONAL_SLEW_RATE = 1; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double TRACK_WIDTH = Units.inchesToMeters(27.5);
    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(24.75);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
    public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
    public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int FRONT_LEFT_DRIVING_CAN_ID = 11;
    public static final int REAR_LEFT_DRIVING_CAN_ID = 13;
    public static final int FRONT_RIGHT_DRIVING_CAN_ID = 15;
    public static final int REAR_RIGHT_DRIVING_CAN_ID = 17;

    public static final int FRONT_LEFT_TURNING_CAN_ID = 12;
    public static final int REAR_LEFT_TURNING_CAN_ID = 14;
    public static final int FRONT_RIGHT_TURNING_CAN_ID = 16;
    public static final int REAR_RIGHT_TURNING_CAN_ID = 18;

    public static final boolean GYRO_REVERSED = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int DRIVING_MOTOR_PINION_TEETH = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean TURNING_ENCODER_INVERTED = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.FREE_SPEED_RPM / 60;
    public static final double WHEEL_DIAMETER_METERS = 0.072;
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
    public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
        / DRIVING_MOTOR_REDUCTION;

    public static final double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI)
        / DRIVING_MOTOR_REDUCTION; // meters
    public static final double DRIVING_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METERS * Math.PI)
        / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second

    public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
    public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

    public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
    public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

    public static final double DRIVING_P = 0.04;
    public static final double DRIVING_I = 0;
    public static final double DRIVING_D = 0;
    public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
    public static final double DRIVING_MIN_OUTPUT = -1;
    public static final double DRIVING_MAX_OUTPUT = 1;

    public static final double TURNING_P = 1;
    public static final double TURNING_I = 0;
    public static final double TURNING_D = 0;
    public static final double TURNING_FF = 0;
    public static final double TURNING_MIN_OUTPUT = -1;
    public static final double TURNING_MAX_OUTPUT = 1;

    public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

    public static final int DRIVING_MOTOR_CURRENT_LIMIT = 10; // amps (was 50)
    public static final int TURNING_MOTOR_CURRENT_LIMIT = 5; // amps (was 20)
  }

  public static final class NeoMotorConstants {
    public static final double FREE_SPEED_RPM = 2838;
  }
}