package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPosition;

public class ArmSubsystem extends SubsystemBase {
   
    private final WPI_TalonSRX talonMotorLeft;    
    private final WPI_TalonSRX talonMotorRight;   
    //private final WPI_TalonSRX talonMotorCenter;
    private final Encoder throughBoreEncoder; 
    private double speed;
    private boolean printDebug;
    private boolean printDebugLimitSwitches = false;
    public ArmPosition armPosition = ArmPosition.UNKNOWN;
  
    public ArmSubsystem (int motorIDInputLeft, int motorIDInputRight, double speed, boolean printDebugInput) {

        this.speed = speed;
        printDebug = printDebugInput;

        talonMotorLeft = new WPI_TalonSRX(motorIDInputLeft);
        talonMotorRight = new WPI_TalonSRX(motorIDInputRight);
        //talonMotorCenter = new WPI_TalonSRX(motorIDInputCenter);

        // This is the setting for moving arm downward, which will happen when speed is greater than 0
        talonMotorLeft.setInverted(true);        
        talonMotorRight.setInverted(false);   
        
        talonMotorRight.setNeutralMode(NeutralMode.Brake);
        talonMotorLeft.setNeutralMode(NeutralMode.Brake);
        //talonMotorCenter.setNeutralMode(NeutralMode.Brake);
        
        // Initializes an encoder on DIO pins 0 and 1
        // 2X encoding and inverted
        throughBoreEncoder = new Encoder(ArmConstants.ARM_BORE_ENCODER_CHANNEL_A_DIO, ArmConstants.ARM_BORE_ENCODER_CHANNEL_B_DIO, true, Encoder.EncodingType.k2X);
        throughBoreEncoder.setDistancePerPulse((double) 360/8192/2);

        if (printDebug) {
            System.out.println("ArmSubsystem: MotorID constructor ");
        }
    }

    public void InitializeEncoder() {
        throughBoreEncoder.reset();

    }

    public void ReverseLimitPosition(boolean initializeEncoder) {
        boolean speedSet = false;
        int BoreEncoderVal;
        int ls_left_fwd, ls_left_rev, ls_right_fwd, ls_right_rev;

        // FIXME: Update once other limit switch is in place
        //while ((talonMotorLeft.isFwdLimitSwitchClosed()!=1) && (talonMotorRight.isRevLimitSwitchClosed()!=1)) {
        //while (talonMotorLeft.isRevLimitSwitchClosed()!=1) {
        while (talonMotorRight.isFwdLimitSwitchClosed()!=1) {
            if (speedSet==false) {
                // Negative speed means moving arm upward/reverse
                talonMotorLeft.set(-speed);
                talonMotorRight.set(-speed);
                speedSet=true;
                }
            if (printDebugLimitSwitches) {
                ls_left_fwd=talonMotorLeft.isFwdLimitSwitchClosed();
                ls_left_rev=talonMotorLeft.isRevLimitSwitchClosed();
                ls_right_fwd=talonMotorRight.isFwdLimitSwitchClosed();
                ls_right_rev=talonMotorRight.isRevLimitSwitchClosed();
                System.out.println("Limit Switches: ls_left_fwd:"+ls_left_fwd+
                                    " ls_left_rev:"+ls_left_rev+
                                    " ls_right_fwd:"+ls_right_fwd+
                                    " ls_right_rev"+ls_right_rev);
            }  
        }
       
        // Stop motors in case one of them still running
        talonMotorLeft.set(0.0);
        talonMotorRight.set(0.0);
    
        if (initializeEncoder) {
            // Initialize (zero) the Bore Encoder
            throughBoreEncoder.reset(); 
        }

        if (printDebug) {
            BoreEncoderVal=throughBoreEncoder.getRaw();
            System.out.println("ReverseLimitPosition BoreEncoderVal End = "+BoreEncoderVal);
        }

        armPosition = ArmPosition.REVERSE_LIMIT;

    }

    public void ForwardLimitPosition() {
        boolean speedSet = false;
        int BoreEncoderVal;
        int ls_left_fwd, ls_left_rev, ls_right_fwd, ls_right_rev;

        //while ((talonMotorLeft.isFwdLimitSwitchClosed()!=1) || (talonMotorRight.isRevLimitSwitchClosed()!=1)) {
        while (talonMotorLeft.isFwdLimitSwitchClosed()!=1) {
            if (speedSet==false) {
                // Positive speed means moving arm downward/forward
                talonMotorLeft.set(speed);
                talonMotorRight.set(speed);
                speedSet=true;
            }
            if (printDebugLimitSwitches) {
                ls_left_fwd=talonMotorLeft.isFwdLimitSwitchClosed();
                ls_left_rev=talonMotorLeft.isRevLimitSwitchClosed();
                ls_right_fwd=talonMotorRight.isFwdLimitSwitchClosed();
                ls_right_rev=talonMotorRight.isRevLimitSwitchClosed();
                System.out.println("Limit Switches: ls_left_fwd:"+ls_left_fwd+
                                        " ls_left_rev:"+ls_left_rev+
                                        " ls_right_fwd:"+ls_right_fwd+
                                        " ls_right_rev"+ls_right_rev);
                
                }  
            }
                   
        // Stop motors in case one of them still running
        talonMotorLeft.set(0.0);
        talonMotorRight.set(0.0);

        //if (printDebug) {
        if (true) {
            BoreEncoderVal=throughBoreEncoder.getRaw();
            System.out.println("ForwardLimitPosition BoreEncoderVal End = "+BoreEncoderVal);
        }

        armPosition = ArmPosition.FORWARD_LIMIT;

    }

    /* Sets arm to upright position which is for start of game */
    public void UprightPosition() {
        int BoreEncoderVal;
        boolean speedSet = false;

        //SmartDashboard.putNumber("arm encoder direction",throughBoreEncoder.getRaw());

        if (printDebug) {
             System.out.println("UprightPosition");
        }
        
        // Run until ARM_UPRIGHT_BORE_ENCODER_POSITION
        BoreEncoderVal=throughBoreEncoder.getRaw();
        if (printDebug) {
            System.out.println("UprightPosition BoreEncoderVal Start = "+BoreEncoderVal);
            System.out.println("UprightPosition BoreEncoderVal Start = "+throughBoreEncoder.getDistance());
        }

        if (BoreEncoderVal>ArmConstants.ARM_UPRIGHT_BORE_ENCODER_POSITION) {
            while (BoreEncoderVal>ArmConstants.ARM_UPRIGHT_BORE_ENCODER_POSITION) {
                if (speedSet==false) {
                    talonMotorLeft.set(-speed);
                    talonMotorRight.set(-speed);
                    speedSet=true;
                }
            BoreEncoderVal=throughBoreEncoder.getRaw();
            }
        }
        else {
            while (BoreEncoderVal<ArmConstants.ARM_UPRIGHT_BORE_ENCODER_POSITION) {
                if (speedSet==false) {
                    talonMotorLeft.set(speed);
                    talonMotorRight.set(speed);
                    speedSet=true;
                }
            BoreEncoderVal=throughBoreEncoder.getRaw();   
            }         
        }

        talonMotorLeft.set(0);
        talonMotorRight.set(0);

        if (printDebug) {
            BoreEncoderVal=throughBoreEncoder.getRaw();
            System.out.println("UprightPosition BoreEncoderVal End = "+BoreEncoderVal);
            System.out.println("UprightPosition BoreEncoderVal End = "+throughBoreEncoder.getDistance());
        }

        armPosition = ArmPosition.UPRIGHT;

    }

    /* Sets arm to intake position to take in game piece */
    public void IntakePosition() {
        int BoreEncoderVal;
        boolean speedSet = false;

        if (printDebug) {
            System.out.println("IntakePosition");
        }

        BoreEncoderVal=throughBoreEncoder.getRaw();
        if (printDebug) {
            System.out.println("IntakePosition BoreEncoderVal Start = "+BoreEncoderVal);
            System.out.println("IntakePosition BoreEncoderVal Start = "+throughBoreEncoder.getDistance());
        }

        // Run until ARM_INTAKE_BORE_ENCODER_POSITION
        if (BoreEncoderVal>ArmConstants.ARM_INTAKE_BORE_ENCODER_POSITION) {
            while (BoreEncoderVal>ArmConstants.ARM_INTAKE_BORE_ENCODER_POSITION) {
                if (speedSet==false) {
                    talonMotorLeft.set(-speed);
                    talonMotorRight.set(-speed);
                    speedSet=true;
                }
            BoreEncoderVal=throughBoreEncoder.getRaw();
            }
        }
        else {
            while (BoreEncoderVal<ArmConstants.ARM_INTAKE_BORE_ENCODER_POSITION) {
                if (speedSet==false) {
                    talonMotorLeft.set(speed);
                    talonMotorRight.set(speed);
                    speedSet=true;
                }
            BoreEncoderVal=throughBoreEncoder.getRaw();   
            }         
        }

        talonMotorLeft.set(0);
        talonMotorRight.set(0);

        if (printDebug) {
            BoreEncoderVal=throughBoreEncoder.getRaw();
            System.out.println("IntakePosition BoreEncoderVal End = "+BoreEncoderVal);
            System.out.println("IntakePosition BoreEncoderVal End = "+throughBoreEncoder.getDistance());
        }    

        armPosition = ArmPosition.INTAKE;
        
    }

    /* Sets arm to shooter position to spit out game piece */
    public void SpeakerShooterPosition() {
        int BoreEncoderVal;
        boolean speedSet = false;

        if (printDebug) {
            System.out.println("SpeakerShooterPosition");
        }

        BoreEncoderVal=throughBoreEncoder.getRaw();
        if (printDebug) {
            System.out.println("SpeakerShooterPosition BoreEncoderVal Start = "+BoreEncoderVal);
            System.out.println("SpeakerShooterPosition BoreEncoderVal Start = "+throughBoreEncoder.getDistance());
        }

        // Run until ARM_SPEAKER_SHOOTER_BORE_ENCODER_POSITION
        if (BoreEncoderVal>ArmConstants.ARM_SPEAKER_SHOOTER_BORE_ENCODER_POSITION) {
            while (BoreEncoderVal>ArmConstants.ARM_SPEAKER_SHOOTER_BORE_ENCODER_POSITION) {
                if (speedSet==false) {
                    talonMotorLeft.set(-speed);
                    talonMotorRight.set(-speed);
                    speedSet=true;
                }
            BoreEncoderVal=throughBoreEncoder.getRaw();
            }
        }
        else {
            while (BoreEncoderVal<ArmConstants.ARM_SPEAKER_SHOOTER_BORE_ENCODER_POSITION) {
                if (speedSet==false) {
                    talonMotorLeft.set(speed);
                    talonMotorRight.set(speed);
                    speedSet=true;
                }
            BoreEncoderVal=throughBoreEncoder.getRaw();  
            }          
        }

        talonMotorLeft.set(0);
        talonMotorRight.set(0);

        if (printDebug) {
            BoreEncoderVal=throughBoreEncoder.getRaw();
            System.out.println("SpeakerShooterPosition BoreEncoderVal End = "+BoreEncoderVal);
            System.out.println("SpeakerShooterPosition BoreEncoderVal End = "+throughBoreEncoder.getDistance());
        }    

        armPosition = ArmPosition.SPEAKER_SHOOTER;

    }

    /* Sets arm to shooter position to spit out game piece */
    public void AmpShooterPosition() {
        int BoreEncoderVal;
        boolean speedSet = false;

        if (printDebug) {
            System.out.println("AmpShooterPosition");
        }

        BoreEncoderVal=throughBoreEncoder.getRaw();
        if (printDebug) {
            System.out.println("AmpShooterPosition BoreEncoderVal Start = "+BoreEncoderVal);
        }

        // Run until ARM_AMP_SHOOTER_BORE_ENCODER_POSITION
        if (BoreEncoderVal>ArmConstants.ARM_AMP_SHOOTER_BORE_ENCODER_POSITION) {
            while (BoreEncoderVal>ArmConstants.ARM_AMP_SHOOTER_BORE_ENCODER_POSITION) {
                if (speedSet==false) {
                    talonMotorLeft.set(-speed);
                    talonMotorRight.set(-speed);
                    speedSet=true;
                }
            BoreEncoderVal=throughBoreEncoder.getRaw();
            }
        }
        else {
            while (BoreEncoderVal<ArmConstants.ARM_AMP_SHOOTER_BORE_ENCODER_POSITION) {
                if (speedSet==false) {
                    talonMotorLeft.set(speed);
                    talonMotorRight.set(speed);
                    speedSet=true;
                }
            BoreEncoderVal=throughBoreEncoder.getRaw(); 
            }           
        }         

        talonMotorLeft.set(0);
        talonMotorRight.set(0);

        if (printDebug) {
            BoreEncoderVal=throughBoreEncoder.getRaw();
            System.out.println("AmpSpeakerShooterPosition BoreEncoderVal End = "+BoreEncoderVal);
        }    

        armPosition = ArmPosition.AMP_SHOOTER;

    }

    /* Sets arm to handing-on-chain position which is for end of game */
    public void HangPosition() {
        int BoreEncoderVal;
        boolean speedSet = false;

        if (printDebug) {
            System.out.println("HangPosition");
        }

        BoreEncoderVal=throughBoreEncoder.getRaw();
        if (printDebug) {
            System.out.println("HangPosition BoreEncoderVal Start = "+BoreEncoderVal);
        }

        // Run until ARM_HANG_BORE_ENCODER_POSITION
        if (BoreEncoderVal>ArmConstants.ARM_HANG_BORE_ENCODER_POSITION) {
            while (BoreEncoderVal>ArmConstants.ARM_HANG_BORE_ENCODER_POSITION) {
                if (speedSet==false) {
                    talonMotorLeft.set(-speed);
                    talonMotorRight.set(-speed);
                    speedSet=true;
                }
            BoreEncoderVal=throughBoreEncoder.getRaw();
            }
        }
        else {
            while (BoreEncoderVal<ArmConstants.ARM_HANG_BORE_ENCODER_POSITION) {
                if (speedSet==false) {
                    talonMotorLeft.set(speed);
                    talonMotorRight.set(speed);
                    speedSet=true;
                }
            BoreEncoderVal=throughBoreEncoder.getRaw();    
            }        
        }  

        talonMotorLeft.set(0);
        talonMotorRight.set(0);

        if (printDebug) {
            BoreEncoderVal=throughBoreEncoder.getRaw();
            System.out.println("HangPosition BoreEncoderVal End = "+BoreEncoderVal);
        }    

        armPosition = ArmPosition.HANG;

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public boolean isPrintDebug() {
        return printDebug;
    }

    public void setPrintDebug(boolean printDebug) {
        this.printDebug = printDebug;
    }

}

