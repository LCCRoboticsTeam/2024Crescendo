package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
   
    private final WPI_TalonSRX talonMotorLeft;   
    private final WPI_TalonSRX talonMotorRight;
    //private final Encoder throughBoreEncoderLeft;
    //private final Encoder throughBoreEncoderRight;
    private double speed;
  
    public ShooterSubsystem (int motorIDInputLeft, int motorIDInputRight, double speed) {
    
        this.speed = speed;

        talonMotorLeft = new WPI_TalonSRX(motorIDInputLeft);
        talonMotorLeft.setInverted(false);        
        talonMotorRight = new WPI_TalonSRX(motorIDInputRight);
        talonMotorRight.setInverted(false);
    }

    /* Sets Shooter to take in game piece */
    public void ShooterIn() {
        //talonMotorLeft.setInverted(true);
        //talonMotorRight.setInverted(true);

        talonMotorLeft.set(-speed);
        talonMotorRight.set(-speed);

    }

    /* Sets Shooter to send game piece */
    public void ShooterOut(boolean highspeed) {
        // int BoreEncoderValLeft;
        // int BoreEncoderValRight;

        //talonMotorLeft.setInverted(false);
        //talonMotorRight.setInverted(false);

        if (highspeed) {
            talonMotorLeft.set(speed*ShooterConstants.SHOOTER_HIGH_SPEED_MULTIPLIER);
            talonMotorRight.set(speed*ShooterConstants.SHOOTER_HIGH_SPEED_MULTIPLIER);
        }
        else {
          talonMotorLeft.set(speed);
          talonMotorRight.set(speed);
        }

        //BoreEncoderValLeft=throughBoreEncoderLeft.getRate();
        //BoreEncoderValRight=throughBoreEncoderRight.getRate()
        //if (printDebug) {
        //    System.out.println("intakeIn BoreEncoderValLeft = "+BoreEncoderValLeft);
        //    System.out.println("intakeIn BoreEncoderValRight = "+BoreEncoderValRight);
        //}

    }

    /* Turn Shooter off */
    public void ShooterOff() {
        talonMotorLeft.set(0.0);
        talonMotorRight.set(0.0);

    }

}

