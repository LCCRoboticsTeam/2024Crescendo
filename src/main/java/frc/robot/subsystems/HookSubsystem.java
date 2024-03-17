// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.HookConstants.SPEED;

public class HookSubsystem extends SubsystemBase {

  private final WPI_TalonSRX talonMotor;
  private final WPI_TalonSRX solenoid;

  private boolean solenoidUnlocked;

  /** Creates a new HookSubsystem. */
  public HookSubsystem(int motorID, int solenoidID) {
    solenoidUnlocked = true;
    talonMotor = new WPI_TalonSRX(motorID);
    solenoid = new WPI_TalonSRX(solenoidID);
  }

  public void moveUp() {
    talonMotor.set(SPEED);
  }

  public void moveDown() {
    talonMotor.set(-SPEED);
  }

  public void moveStop() {
    talonMotor.set(0.0);
  }

  public void setSolenoidState(boolean unlocked) {
    solenoidUnlocked = unlocked;
  }

  private void lockSolenoid() {
    solenoid.set(0.0);
  }

  private void unlockSolenoid() {
    solenoid.set(1.0);
  }

  public boolean limitSwitchClosed() {
    return talonMotor.isFwdLimitSwitchClosed() == 1;
  }

  @Override
  public void periodic() {
    if (solenoidUnlocked) {
      unlockSolenoid();
    } else {
      lockSolenoid();
    }
  }

}
