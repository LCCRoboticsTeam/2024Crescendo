package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
   
    private final WPI_TalonSRX talonMotor;
    //private final Encoder throughBoreEncoder;
    DigitalInput BeamBreak; 
    private double speed;
    private boolean printDebug;
  
    public IntakeSubsystem (int motorIDInput, double speed, boolean printDebugInput) {

        this.speed = speed;
        printDebug = printDebugInput;

        talonMotor = new WPI_TalonSRX(motorIDInput);
        BeamBreak = new DigitalInput(IntakeConstants.INTAKE_BEAM_BREAK_DIO); 

        // Positive speed value is for intakeIn
        talonMotor.setInverted(false);        

        if (printDebug) {
            System.out.println("intakesubsystem: MotorID constructor ");
        }
    }

    /* Sets intake to take in game piece */
    public void intakeIn() {
        // int BoreEncoderVal;

        //talonMotor.setInverted(false);
        talonMotor.set(-speed);

        //BoreEncoderVal=throughBoreEncoder.getRate();
        //if (printDebug) {
        //    System.out.println("intakeIn BoreEncoderVal = "+BoreEncoderVal);
        //}

    }

    /* Sets intake to spit out game piece */
    public void intakeOut() {
        //talonMotor.setInverted(true);
        talonMotor.set(speed);

    }

    /* Turn intake off */
    public void intakeOff() {
        talonMotor.set(0.0);

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

