package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
   
    private final WPI_TalonSRX talonMotor;
    private LaserCan LC_0;
    private LaserCan LC_1;
    //private final Encoder throughBoreEncoder;
    private double speed;
  
    public IntakeSubsystem (int motorIDInput, int LC_0_ID, int LC_1_ID, double speed) {

        this.speed = speed;

        talonMotor = new WPI_TalonSRX(motorIDInput);

        // Initializes an encoder on DIO pins 0 and 1
        // 2X encoding and inverted
        //throughBoreEncoder = new Encoder(ArmConstants.ARM_BORE_ENCODER_CHANNEL_A_DIO,
        //        ArmConstants.ARM_BORE_ENCODER_CHANNEL_B_DIO, true, Encoder.EncodingType.k2X);
        //throughBoreEncoder.setDistancePerPulse((double) 360 / 8192 / 2);

        // Configures the encoder to return a distance of 4 for every 256 pulses
        // Also changes the units of getRate
        // throughBoreEncoder.setDistancePerPulse(4.0/256.0);

        // Configures the encoder to consider itself stopped after .1 seconds
        // throughBoreEncoder.setMaxPeriod(0.1);

        // Configures the encoder to consider itself stopped when its rate is below 10
        // throughBoreEncoder.setMinRate(10);

        // Reverses the direction of the encoder
        // throughBoreEncoder.setReverseDirection(true);

        // Configures an encoder to average its period measurement over 5 samples
        // Can be between 1 and 127 samples
        //throughBoreEncoder.setSamplesToAverage(5);

        LC_0 = new LaserCan(LC_0_ID);
        try {
            LC_0.setRangingMode(LaserCan.RangingMode.SHORT);
            LC_0.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            LC_0.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
          } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
          }
        LC_1 = new LaserCan(LC_1_ID);
        try {
            LC_1.setRangingMode(LaserCan.RangingMode.SHORT);
            LC_1.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            LC_1.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
          } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
          }

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

    public boolean noteDetected() {
        LaserCan.Measurement measurement = LC_0.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            //System.out.println("The target is " + measurement.distance_mm + "mm away!");
            return true;
          } else {
            //System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
            return false;
          }
    }

    public int getBoreEncoderRate() {
        //return throughBoreEncoder.getRate();
        return 2;
    }

    public void InitializeEncoder() {
        //throughBoreEncoder.reset();

    }

}

