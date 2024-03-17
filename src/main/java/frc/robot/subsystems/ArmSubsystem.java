package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPosition;
import frc.robot.commands.ArmToForwardLimitCommand;

public class ArmSubsystem extends SubsystemBase {

    private final WPI_TalonSRX talonMotorLeft;
    private final WPI_TalonSRX talonMotorRight;
    // private final WPI_TalonSRX talonMotorCenter;
    private final Encoder throughBoreEncoder;
    private double speed;
    private ArmPosition armPosition;

    public ArmSubsystem(int motorIDInputLeft, int motorIDInputRight, double speed) {

        armPosition = ArmPosition.UNKNOWN;

        this.speed = speed;

        talonMotorLeft = new WPI_TalonSRX(motorIDInputLeft);
        talonMotorRight = new WPI_TalonSRX(motorIDInputRight);

        // This is the setting for moving arm downward, which will happen when speed is
        // greater than 0
        talonMotorLeft.setInverted(true);
        talonMotorRight.setInverted(false);

        talonMotorRight.setNeutralMode(NeutralMode.Brake);
        talonMotorLeft.setNeutralMode(NeutralMode.Brake);

        // Initializes an encoder on DIO pins 0 and 1
        // 2X encoding and inverted
        throughBoreEncoder = new Encoder(ArmConstants.ARM_BORE_ENCODER_CHANNEL_A_DIO,
                ArmConstants.ARM_BORE_ENCODER_CHANNEL_B_DIO, true, Encoder.EncodingType.k2X);
        throughBoreEncoder.setDistancePerPulse((double) 360 / 8192 / 2);

        new ArmToForwardLimitCommand(this).schedule();
    }

    public ArmPosition getArmPosition() {
        return armPosition;
    }

    public void setArmPosition(ArmPosition armPosition) {
        this.armPosition = armPosition;
    }

    public int getBoreEncoderVal() {
        return throughBoreEncoder.getRaw();
    }

    public void InitializeEncoder() {
        throughBoreEncoder.reset();

    }

    public void moveArmUp() {
        talonMotorLeft.set(-speed);
        talonMotorRight.set(-speed);
    }

    public void moveArmDown() {
        talonMotorLeft.set(speed);
        talonMotorRight.set(speed);
    }

    public void moveStop() {
        talonMotorLeft.set(0);
        talonMotorRight.set(0);
    }

    public void resetEncoder() {
        throughBoreEncoder.reset();
    }

    public boolean isReverseLimitSwitchClosed() {
        return talonMotorRight.isFwdLimitSwitchClosed() == 1;
    }

    public boolean isForwardLimitSwitchClosed() {
        return talonMotorLeft.isFwdLimitSwitchClosed() == 1;
    }

}
