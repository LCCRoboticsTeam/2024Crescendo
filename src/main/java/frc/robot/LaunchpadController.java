// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/** Add your docs here. */
public class LaunchpadController extends Joystick {

    private static final int ARM_UP = 8;
    private static final int ARM_DOWN = 9;
    private static final int INTAKE_IN = 10;
    private static final int SHOOTER_OUT = 11;
    private static final int SAFETY = 4;
    private static final int CLIMB_UP = 7;
    private static final int CLIMB_DOWN = 6;
    private static final int MISC_BLUE = 5;

    public LaunchpadController(int port) {
        super(port);
    }

    public boolean getArmUp() {
        return getRawButton(ARM_UP);
    }

    public boolean getArmDown() {
        return getRawButton(ARM_DOWN);
    }

    public boolean getIntakeIn() {
        return getRawButton(INTAKE_IN);
    }

    public boolean getShooterOut() {
        return getRawButton(SHOOTER_OUT);
    }

    public boolean getSafety() {
        return getRawButton(SAFETY);
    }

    public boolean getClimbUp() {
        return getRawButton(CLIMB_UP);
    }

    public boolean getClimbDown() {
        return getRawButton(CLIMB_DOWN);
    }

    public boolean getMiscBlue() {
        return getRawButton(MISC_BLUE);
    }

}
