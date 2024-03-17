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
  
    public IntakeSubsystem (int motorIDInput, double speed) {

        this.speed = speed;

        talonMotor = new WPI_TalonSRX(motorIDInput);
        BeamBreak = new DigitalInput(IntakeConstants.INTAKE_BEAM_BREAK_DIO); 

        // Positive speed value is for intakeIn
        talonMotor.setInverted(false);
    }

    /* Sets intake to take in game piece */
    public void intakeIn() {
        talonMotor.set(-speed);
    }

    /* Sets intake to spit out game piece */
    public void intakeOut() {
        talonMotor.set(speed);
    }

    /* Turn intake off */
    public void intakeOff() {
        talonMotor.set(0.0);

    }

}

