package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPosition;
import frc.robot.commands.ArmToForwardLimitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {

    public final WPI_TalonSRX talonMotorLeft;
    public final WPI_TalonSRX talonMotorRight;
    // private final WPI_TalonSRX talonMotorCenter;
    private final Encoder throughBoreEncoder;
    private double speed_up;
    private double speed_down;
    private ArmPosition armPosition;

    public ArmSubsystem(int motorIDInputLeft, int motorIDInputRight, double speed_up, double speed_down) {
        
        armPosition = ArmPosition.UNKNOWN;
        
        this.speed_up = speed_up;
        this.speed_down = speed_down;

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
        talonMotorLeft.set(-speed_up);
        talonMotorRight.set(-speed_up);

        //talonMotorLeft.set(0);
        //talonMotorRight.set(0);
        //System.out.println("Right ARM Reverse Limit Closed = "+talonMotorRight.isRevLimitSwitchClosed());
        //System.out.println("Left ARM Reverse Limit Closed = "+talonMotorLeft.isRevLimitSwitchClosed());
        //System.out.println("Right ARM Forward Limit Closed = "+talonMotorRight.isFwdLimitSwitchClosed());
        //System.out.println("Left ARM Forward Limit Closed = "+talonMotorLeft.isFwdLimitSwitchClosed());
    }

    public void moveArmUpHalfSpeed() {
        talonMotorLeft.set(-speed_up/2);
        talonMotorRight.set(-speed_up/2);
    }

    public void moveArmDown() {
        talonMotorLeft.set(speed_down);
        talonMotorRight.set(speed_down);

        //talonMotorLeft.set(0);
        //talonMotorRight.set(0);
        //System.out.println("Right ARM Reverse Limit Closed = "+talonMotorRight.isRevLimitSwitchClosed());
        //System.out.println("Left ARM Reverse Limit Closed = "+talonMotorLeft.isRevLimitSwitchClosed());
        //System.out.println("Right ARM Forward Limit Closed = "+talonMotorRight.isFwdLimitSwitchClosed());
        //System.out.println("Left ARM Forward Limit Closed = "+talonMotorLeft.isFwdLimitSwitchClosed());
    }

    public void holdArmUp() {
        talonMotorLeft.set(-ArmConstants.ARM_MOTORP_SPEED_HOLD);
        talonMotorRight.set(-ArmConstants.ARM_MOTORP_SPEED_HOLD);
    }

    public void moveStop() {
        talonMotorLeft.set(0);
        talonMotorRight.set(0);
    }

    public void resetEncoder() {
        throughBoreEncoder.reset();
    }
 
    public boolean isReverseLimitSwitchClosed() {
        //return (talonMotorRight.isRevLimitSwitchClosed() == 1);
        //return (talonMotorLeft.isRevLimitSwitchClosed() == 1);
        return (talonMotorRight.isRevLimitSwitchClosed() == 1) || (talonMotorLeft.isRevLimitSwitchClosed() == 1);
    }

    public boolean isForwardLimitSwitchClosed() {
        //return (talonMotorRight.isFwdLimitSwitchClosed() == 1);
        //return (talonMotorLeft.isFwdLimitSwitchClosed() == 1);
        return (talonMotorRight.isFwdLimitSwitchClosed() == 1) || (talonMotorLeft.isFwdLimitSwitchClosed() == 1);
    }

    


}
